"""
LLM Planner Node for Cocoa Speech
Receives Intent messages, generates action plans via LLM, executes via Action Server
"""

import os
import json
import re
import time
import threading
import requests
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from cocoa_msgs.msg import Intent, ExecutionFeedback, ThesisMetrics
from cocoa_msgs.action import DroneCommand
from cocoa_msgs.srv import PlanPath
from geometry_msgs.msg import Point
from std_msgs.msg import String

from .tool_definitions import get_tools_prompt
from .openai_planner_interface import PlannerOpenAIInterface, OPENAI_AVAILABLE


class LLMPlannerNode(Node):
    """
    LLM Planner ROS2 Node
    
    Subscribes to Intent messages, uses LLM to generate action plans,ss
    and executes them via Crazyswarm2 services.
    
    Note: Drone state (position, is_flying) is now managed by LGA node.
    """
    
    def __init__(self):
        super().__init__('llm_planner_node')
        
        # Create callback group for service calls
        self.cb_group = ReentrantCallbackGroup()
        
        # Declare parameters (using HTTP server instead of llama_ros)
        self.declare_parameter('drone_id', 'cf231')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('max_tokens', 1024) # Increased for complex multi-intent plans
        self.declare_parameter('intent_topic', '/intent')
        self.declare_parameter('llm_server_url', 'http://localhost:8081')
        self.declare_parameter('use_cot', True)  # Chain-of-Thought flag
        self.declare_parameter('include_raw_command', True)  # Whether to include raw command in prompt
        
        # LLM Backend selection: 'llama' (local) or 'openai' (API)
        self.declare_parameter('llm_backend', 'llama')  # 'llama' or 'openai'
        self.declare_parameter('openai_model', 'gpt-5.2')  # gpt-4o-mini, gpt-4o, gpt-4-turbo
        
        # Get parameters
        self.drone_id = self.get_parameter('drone_id').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        intent_topic = self.get_parameter('intent_topic').value
        self.llm_server_url = self.get_parameter('llm_server_url').value
        self.use_cot = self.get_parameter('use_cot').get_parameter_value().bool_value
        self.include_raw_command = self.get_parameter('include_raw_command').get_parameter_value().bool_value
        self.llm_backend = self.get_parameter('llm_backend').value
        
        self.get_logger().info(f"LLM Planner initialized (Backend: {self.llm_backend})")
        self.get_logger().info(f"Chain-of-Thought enabled: {self.use_cot}")
        
        # Load system prompt template
        self.prompt_template = self._load_prompt_template()
        
        # Load GBNF grammar for constrained JSON generation
        self.grammar = self._load_grammar()
        
        # Current drone state (parsed from LGA context via intent)
        self.is_flying = False
        self.is_armed = False
        self.position = [0.0, 0.0, 0.0]  # Updated from context
        self.yaw = 0.0  # Heading in degrees
        self.state_initialized = False  # Track if we've received state from context
        
        # Initialize OpenAI interface if using openai backend
        self.openai_interface = None
        if self.llm_backend == 'openai':
            if not OPENAI_AVAILABLE:
                self.get_logger().error("OpenAI backend selected but 'openai' package not installed!")
                self.get_logger().error("Install with: pip install openai")
                raise ImportError("openai package required for llm_backend='openai'")
            
            openai_model = self.get_parameter('openai_model').value
            self.get_logger().info(f"Using OpenAI backend with model: {openai_model}")
            self.openai_interface = PlannerOpenAIInterface(
                logger=self.get_logger(),
                model=openai_model,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )
        else:
            # Test llama.cpp server connection
            self.get_logger().info(f"Connecting to llama.cpp HTTP server at {self.llm_server_url}...")
            try:
                resp = requests.get(f"{self.llm_server_url}/health", timeout=5)
                if resp.status_code == 200:
                    self.get_logger().info("Connected to llama.cpp HTTP server")
                else:
                    self.get_logger().warn(f"Server returned status {resp.status_code}")
            except Exception as e:
                self.get_logger().warn(f"Could not connect: {e}")
                self.get_logger().info("Start server with: python -m llama_cpp.server --model <path> --n_gpu_layers 32")
        
        # Initialize action client (replaces blocking ActionExecutor)
        self._action_client = ActionClient(
            self,
            DroneCommand,
            'drone_command',
            callback_group=self.cb_group
        )
        
        # Path planner client for navigate_to actions
        self._path_planner_client = self.create_client(
            PlanPath,
            '/plan_path',
            callback_group=self.cb_group
        )
        
        # Subscribe to Intent topic
        self.intent_sub = self.create_subscription(
            Intent,
            intent_topic,
            self._intent_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Flag to prevent concurrent planning
        self.is_planning = False
        
        # Current intent being processed (for feedback publishing)
        self.current_intent = None
        
        # Metrics for thesis logging
        self.last_planning_metrics = {
            'inference_time_ms': 0,
            'input_tokens': 0,
            'plan_valid': False,
            'action_count': 0
        }
        
        # Publisher for execution feedback
        self.feedback_pub = self.create_publisher(
            ExecutionFeedback,
            '/execution_feedback',
            10
        )
        
        # Publisher for query responses (for QUERY intents like battery status)
        self.response_pub = self.create_publisher(
            String,
            '/drone_response',
            10
        )
        
        # THESIS: Publisher for experiment metrics
        self.metrics_pub = self.create_publisher(
            ThesisMetrics,
            '/thesis_metrics',
            10
        )
        
        self.get_logger().info("="* 50)
        self.get_logger().info("LLM Planner Node Ready (HTTP)")
        self.get_logger().info(f"  Drone: {self.drone_id}")
        self.get_logger().info(f"  Intent topic: {intent_topic}")
        self.get_logger().info(f"  LLM Server: {self.llm_server_url}")
        self.get_logger().info(f"  Feedback topic: /execution_feedback")
        self.get_logger().info(f"  Response topic: /drone_response")
        self.get_logger().info("  State: from LGA via intent context")
        self.get_logger().info("=" * 50)
    def _parse_state_from_context(self, context: str):
        """Parse drone state from LGA context string (sole source of state)"""
        import re
        try:
            # Parse is_flying
            if "is_flying: True" in context:
                self.is_flying = True
            elif "is_flying: False" in context:
                self.is_flying = False
            
            # Parse is_armed
            if "is_armed: True" in context:
                self.is_armed = True
            elif "is_armed: False" in context:
                self.is_armed = False
            
            # Parse drone_pos: (x, y, z)
            match = re.search(r'drone_pos:\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)', context)
            if match:
                self.position[0] = float(match.group(1))
                self.position[1] = float(match.group(2))
                self.position[2] = float(match.group(3))
                if not self.state_initialized:
                    self.state_initialized = True
                    self.get_logger().info(f"State from LGA: pos=({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}), yaw={self.yaw:.1f}, flying={self.is_flying}")
            
            # Parse drone_yaw
            match_yaw = re.search(r'drone_yaw:\s*(-?[\d.]+)', context)
            if match_yaw:
                self.yaw = float(match_yaw.group(1))
        except Exception as e:
            self.get_logger().warn(f"Could not parse state from context: {e}")
    
    def _load_prompt_template(self) -> str:
        """Load the planner prompt template"""
        try:
            pkg_share = get_package_share_directory('cocoa_llm_planner')
            prompt_path = os.path.join(pkg_share, 'prompts', 'planner_prompt.txt')
            with open(prompt_path, 'r') as f:
                return f.read()
        except Exception as e:
            self.get_logger().warn(f"Could not load prompt file: {e}, using default")
            return self._get_default_prompt()
    
    def _load_grammar(self) -> str:
        """Load GBNF grammar for JSON constrained generation."""
        try:
            pkg_share = get_package_share_directory('cocoa_llm_planner')
            grammar_path = os.path.join(pkg_share, 'prompts', 'planner_grammar.gbnf')
            with open(grammar_path, 'r') as f:
                grammar = f.read()
                self.get_logger().info(f"Loaded GBNF grammar ({len(grammar)} chars)")
                return grammar
        except Exception as e:
            self.get_logger().warn(f"Could not load grammar file: {e}, grammar disabled")
            return ""
    
    def _get_default_prompt(self) -> str:
        """Default prompt if file not found"""
        return """You are an action planner for a drone.
You are an action planner for a drone.
Current state: is_flying={is_flying}, position=({pos_x}, {pos_y}, {pos_z}), yaw={yaw}
{tools_prompt}

User command: "{raw_command}"

Intents to fulfill (in order):
{intents}

Generate a SINGLE action plan that fulfills ALL intents above in sequence.
Do NOT duplicate actions - each intent should add only the NEW actions needed.

Output valid JSON action plan:
{{"reasoning": "...", "actions": [{{"tool": "...", "params": {{...}}}}]}}
"""
    
    def _generate_unified_plan(self, raw_command: str, combined_intents: str, conversation_history: str = "") -> list:
        """Generate a single unified action plan for all intents"""
        # Build prompt with all intents combined
        # CRITICAL: If include_raw_command is False, hide the raw command to prevent "Double Planning".
        # We want the Planner to execute the resolved INTENTS, not re-interpret the user's potentially ambiguous text.
        prompt_command = raw_command
        if not self.include_raw_command:
            prompt_command = "[REFER TO INTENTS LIST BELOW - COMMAND ALREADY RESOLVED]"
            
        # Format conversation history section (only non-empty for COMPLEX_TASK)
        history_section = ""
        if conversation_history:
            self.get_logger().info(f"Conversation history: {conversation_history}")
            history_section = f"\nCONVERSATION HISTORY (use for referential commands like 'do that again'):\n{conversation_history}\n"
        
        prompt = self.prompt_template.format(
            is_flying=str(self.is_flying).lower(),
            pos_x=self.position[0],
            pos_y=self.position[1],
            pos_z=self.position[2],

            yaw=self.yaw,
            tools_prompt=get_tools_prompt(),
            intent_type="MULTI-INTENT",  # For backward compat with old template
            raw_command=prompt_command,
            context=combined_intents,  # Old template uses 'context'
            intents=combined_intents,  # New template uses 'intents'
            conversation_history=history_section
        )
        
        # Inject CoT instructions/examples or use default Strict mode
        if self.use_cot:
            # 1. Strip default strict-JSON examples to prevent format confusion
            if "EXAMPLE 1" in prompt:
                prompt = prompt.split("EXAMPLE 1")[0]
            
            # 2. Add CoT-specific Examples and Instructions
            cot_section = (
                "EXAMPLE 1 (CoT) - Takeoff & Navigate:\n"
                "<thought>\n"
                "Intent 1: TAKEOFF.\n"
                "Context says: Take off to 1.0m.\n"
                "Preconditions: is_flying=false. Must take off first.\n"
                "Action 1: takeoff(height=1.0).\n\n"
                "Intent 2: GO_TO_LOCATION shelf_a.\n"
                "Context says: shelf_a at (10.0, 5.0, 1.0).\n"
                "Action 2: navigate_to(x=10.0, y=5.0, z=1.0).\n"
                "</thought>\n"
                "```json\n"
                '{"reasoning": "Takeoff then go to shelf_a", "actions": [{"tool": "takeoff", "params": {"height": 1.0}}, {"tool": "navigate_to", "params": {"x": 10.0, "y": 5.0, "z": 1.0}}]}\n'
                "```\n\n"
                
                "INSTRUCTIONS: First, reason step-by-step in a <thought> block.\n"
            )
            
            if not self.include_raw_command:
                cot_section += (
                    "CRITICAL: The 'Intents' list above has ALREADY resolved the user's command.\n"
                    "You MUST execute the specific targets generated in the Intents list (look for '→ TARGET').\n"
                    "Do NOT re-interpret the User Command.\n"
                )
            
            cot_section += (
                "CRITICAL JSON RULES:\n"
                "1. JSON values must be LITERAL numbers, NOT expressions.\n"
                "   - WRONG: \"z\": 0.99 + 1.0\n"
                "   - CORRECT: \"z\": 1.99\n"
                "2. Compute all arithmetic in <thought>, put final values in JSON.\n"
                "3. Output ONLY the <thought> block followed by ```json block.\n\n"
            )
            cot_section += "Format: <thought>Reasoning...</thought>\n```json\n{...}\n```\nResponse:"
            prompt += cot_section
            
            # FORCE the model to start reasoning by pre-filling the thought tag
            prompt += "<thought>"
            
            # Remove the "Output JSON only" instruction from the very top of the prompt
            prompt = prompt.replace("Output JSON only.", "Plan step-by-step.")
            
        else:
             # Keep default prompt but ensure 'OUTPUT (JSON only):' is at the end if not already handled
             pass
        
        # Calculate approximate input tokens
        input_tokens = len(prompt) // 4
        
        # DEBUG: Log the full prompt to understand what the model sees
        self.get_logger().debug(f"Unified prompt:\n{prompt}")
        
        # Branch based on LLM backend
        if self.llm_backend == 'openai' and self.openai_interface:
            # Use OpenAI API
            self.get_logger().info("Sending unified prompt to OpenAI API...")
            
            # For OpenAI, we split system and user content
            # The prompt_template is system, the intents/context is user
            raw_text = self.openai_interface.generate_plan(
                system_prompt=self.prompt_template,
                user_prompt=prompt.replace(self.prompt_template, "").strip(),
                use_cot=self.use_cot
            )
            
            if not raw_text:
                self.last_planning_metrics = self.openai_interface.last_metrics
                return []
            
            # Parse the plan
            actions = self._parse_plan(raw_text)
            
            # Update metrics
            self.openai_interface.update_action_count(len(actions))
            self.last_planning_metrics = self.openai_interface.last_metrics
            self.last_planning_metrics['llm_response'] = raw_text
            
            return actions
        else:
            # Use llama.cpp HTTP server (Chat API)
            try:
                self.get_logger().info("Sending unified prompt to llama.cpp HTTP server (Chat API)...")
                
                # Track inference time
                start_time = time.time()
                
                # Build Chat API messages
                # We wrap the entire formatted prompt (which includes system instructions and context)
                # into the user message, or split it if we want to be pedantic.
                # For Qwen Instruct, putting instructions in System is better, but our prompt string is already mixed.
                # Strategy: Generic System + Full Prompt in User
                messages = [
                    {"role": "system", "content": "You are an autonomous drone action planner. Follow the instructions in the prompt strictly."},
                    {"role": "user", "content": prompt}
                ]
                
                # Calculate tokens
                input_tokens = len(prompt) // 4
                
                # Build request payload
                request_payload = {
                    "messages": messages,
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "stop": ["\n\n\n", "User:", "<|im_end|>"],
                    # "response_format": {"type": "json_object"} # Optional if grammar not used
                }
                
                # Disable grammar if CoT is used (Chat API can support grammar but usually conflicts with free-form CoT)
                if self.grammar and not self.use_cot:
                    # Note: Llama.cpp server Chat API supports 'grammar' field
                     request_payload["grammar"] = self.grammar
                     self.get_logger().info("Using GBNF grammar for constrained JSON output")
                
                response = requests.post(
                    f"{self.llm_server_url}/v1/chat/completions",
                    json=request_payload,
                    timeout=120
                )
                
                # Calculate inference time in milliseconds
                inference_time_ms = int((time.time() - start_time) * 1000)
                
                if response.status_code != 200:
                    self.get_logger().error(f"HTTP error: {response.status_code}")
                    self.get_logger().error(f"Response body: {response.text[:500]}")
                    self.last_planning_metrics = {
                        'inference_time_ms': inference_time_ms,
                        'input_tokens': input_tokens,
                        'plan_valid': False,
                        'action_count': 0
                    }
                    return []
                
                result = response.json()
                raw_text = result["choices"][0]["message"]["content"].strip()
                
                # Log full response (including <thought> block) for debugging CoT
                self.get_logger().info(f"LLM response:\n{raw_text}")
                self.get_logger().info(f"Planner took {inference_time_ms}ms, ~{input_tokens} tokens")
                
                # Parse the plan
                actions = self._parse_plan(raw_text)
                
                # Store metrics for logging
                self.last_planning_metrics = {
                    'inference_time_ms': inference_time_ms,
                    'input_tokens': input_tokens,
                    'plan_valid': len(actions) > 0,
                    'action_count': len(actions),
                    'llm_response': raw_text
                }
                
                return actions
                
            except requests.exceptions.Timeout:
                self.get_logger().error("LLM request timed out")
                self.last_planning_metrics = {
                    'inference_time_ms': 120000,
                    'input_tokens': input_tokens,
                    'plan_valid': False,
                    'action_count': 0
                }
                return []
            except Exception as e:
                self.get_logger().error(f"LLM inference failed: {e}")
                self.last_planning_metrics = {
                    'inference_time_ms': 0,
                    'input_tokens': input_tokens,
                    'plan_valid': False,
                    'action_count': 0
                }
                return []
    
    def _intent_callback(self, msg: Intent):
        """Process incoming intent(s) and generate unified action plan"""
        if self.is_planning:
            self.get_logger().warn("Already planning, ignoring new intent")
            return
        
        self.is_planning = True
        
        try:
            # New format: msg.intents is an array of SingleIntent
            num_intents = len(msg.intents)
            self.get_logger().info(f"Received {num_intents} intent(s)")
            self.get_logger().info(f"  Command: {msg.raw_command}")
            
            # Extract drone state from first intent's context (from LGA)
            if msg.intents and msg.intents[0].context:
                self._parse_state_from_context(msg.intents[0].context)
            
            self.get_logger().info(f"  Current state: is_flying={self.is_flying}, pos={self.position}")
            
            # ================================================================
            # SPECIAL HANDLING: QUERY intents (battery, status, location, etc.)
            # These don't need the action server - we generate a response
            # directly from the context using the LLM.
            # ================================================================
            query_intents = [i for i in msg.intents if i.intent_type.upper() == 'QUERY']
            if query_intents:
                # Handle all query intents
                for query_intent in query_intents:
                    self._handle_query_intent(query_intent, msg.raw_command)
                
                # If ALL intents are queries, we're done
                non_query_intents = [i for i in msg.intents if i.intent_type.upper() != 'QUERY']
                if not non_query_intents:
                    self.get_logger().info("All intents were QUERY type - no action needed")
                    self.is_planning = False
                    return
                
                # Filter to non-query intents for action planning
                msg.intents = non_query_intents
            
            # Build combined intent description with contexts
            intent_descriptions = []
            for i, single_intent in enumerate(msg.intents):
                self.get_logger().info(f"  Intent [{i+1}/{num_intents}]: {single_intent.intent_type}")
                
                # Get context
                context = single_intent.context if single_intent.context else ""
                
                # Build clear description with action hints
                desc = self._format_intent_for_prompt(i+1, single_intent, context)
                intent_descriptions.append(desc)
            
            # Combine all intents into a single context string
            combined_intents = "\n".join(intent_descriptions)
            
            # Generate unified action plan for ALL intents at once
            all_actions = self._generate_unified_plan(msg.raw_command, combined_intents, msg.conversation_history)
            
            if not all_actions:
                self.get_logger().warn("No actions generated")
                self.is_planning = False
                return
            
            self.get_logger().info(f"Generated unified plan with {len(all_actions)} actions")
            for i, action in enumerate(all_actions):
                self.get_logger().info(f"  [{i+1}] {action.get('tool')}: {action.get('params')}")
            
            # THESIS: Publish planner metrics
            self._publish_planner_metrics(msg.raw_command, len(all_actions))
            
            # Store current intent for feedback publishing
            self.current_intent = msg
            
            # Execute plan via Action Server in background thread
            execution_thread = threading.Thread(
                target=self._execute_plan_in_thread,
                args=(all_actions, msg),
                daemon=True
            )
            execution_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            self.is_planning = False
    
    def _execute_plan_in_thread(self, actions: list, intent: Intent):
        """Wrapper to execute plan and reset is_planning flag when done."""
        try:
            self._execute_plan_via_action(actions, intent)
        finally:
            self.is_planning = False
            self.current_intent = None
    
    def _execute_plan_via_action(self, actions: list, intent: Intent):
        """Execute actions sequentially via the Action Server."""
        import time
        
        # Track results for each action
        action_names = []
        action_results = []
        
        # Wait for action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self._publish_feedback(intent, [], [], False, "Action server not available")
            return
        
        success_count = 0
        total_actions = len(actions)
        overall_success = True
        
        for i, action in enumerate(actions):
            tool_name = action.get('tool', '')
            params = action.get('params', {})
            action_names.append(tool_name)
            
            self.get_logger().info(f"[{i+1}/{total_actions}] Sending goal: {tool_name}")
            
            # ================================================================
            # SPECIAL HANDLING: navigate_to uses path planner
            # ================================================================
            if tool_name == 'navigate_to':
                # Get target coordinates from params (already enriched from intent context)
                target_x = float(params.get('x', 0.0))
                target_y = float(params.get('y', 0.0))
                target_z = float(params.get('z', self.position[2]))  # Use current height as default
                standoff = float(params.get('standoff', 2.0))
                
                self.get_logger().info(f"  Using path planner to navigate to: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                
                # Execute the navigate_to action with coordinates (no EKG query needed!)
                nav_success, nav_message = self._execute_navigate_to(target_x, target_y, target_z, standoff)
                
                if nav_success:
                    action_results.append("SUCCESS")
                    success_count += 1
                else:
                    action_results.append(f"FAILED: {nav_message}")
                    overall_success = False
                    break
                
                continue  # Skip the normal goal execution below
            
            # ================================================================
            # NORMAL ACTIONS: goto, takeoff, land, hover
            # ================================================================
            
            # Build the goal message
            goal = DroneCommand.Goal()
            goal.command_type = tool_name
            goal.target_x = float(params.get('x', 0.0))
            goal.target_y = float(params.get('y', 0.0))
            goal.target_z = float(params.get('z', params.get('height', 1.0)))
            goal.yaw = float(params.get('yaw', 0.0))
            goal.duration = float(params.get('duration', 2.0))
            
            # Send goal
            send_goal_future = self._action_client.send_goal_async(
                goal, 
                feedback_callback=self._feedback_callback
            )
            
            # Wait for goal to be accepted (polling instead of spin)
            while not send_goal_future.done():
                time.sleep(0.01)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal rejected: {tool_name}")
                action_results.append(f"REJECTED")
                overall_success = False
                break
            
            self.get_logger().info(f"Goal accepted: {tool_name}")
            
            # Wait for result (polling instead of spin)
            result_future = goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.01)
            result = result_future.result().result
            
            if result.success:
                self.get_logger().info(f"  Done: {result.message}")
                action_results.append("SUCCESS")
                success_count += 1
            else:
                self.get_logger().error(f"  Failed: {result.message}")
                action_results.append(f"FAILED: {result.message}")
                overall_success = False
                break  # Stop on first failure
        
        summary = f"{success_count}/{total_actions} actions succeeded"
        self.get_logger().info(f"Plan execution: {summary}")
        
        # Publish execution feedback
        self._publish_feedback(intent, action_names, action_results, overall_success, summary)
    
    def _publish_feedback(self, intent: Intent, actions: list, results: list, 
                          success: bool, summary: str):
        """Publish execution feedback for the intent extractor."""
        msg = ExecutionFeedback()
        msg.command = intent.raw_command
        # New Intent format: get intent types from intents array
        if hasattr(intent, 'intents') and intent.intents:
            msg.intent_type = ",".join([i.intent_type for i in intent.intents])
        else:
            msg.intent_type = "UNKNOWN"
        msg.success = success
        msg.actions = actions
        msg.results = results
        msg.summary = summary
        msg.stamp = self.get_clock().now().to_msg()
        
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"Published execution feedback: {summary}")
    
    def _publish_planner_metrics(self, command, action_count):
        """
        Publish planner metrics for thesis test runner.
        
        Args:
            command: Original voice command
            action_count: Number of actions in the plan
        """
        metrics_msg = ThesisMetrics()
        metrics_msg.test_command = command
        metrics_msg.source = "planner"
        metrics_msg.mode = ""  # Not applicable for planner
        
        # Intent extractor fills these
        metrics_msg.intent_type = ""
        metrics_msg.target = ""
        metrics_msg.context_tokens = 0  # Planner doesn't set intent context tokens
        metrics_msg.intent_time_ms = 0
        metrics_msg.planner_tokens = self.last_planning_metrics.get('input_tokens', 0)
        metrics_msg.planner_time_ms = self.last_planning_metrics.get('inference_time_ms', 0)
        metrics_msg.action_count = action_count
        metrics_msg.llm_response = self.last_planning_metrics.get('llm_response', '')
        
        # Timestamp
        metrics_msg.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self.metrics_pub.publish(metrics_msg)
        self.get_logger().info(
            f"[THESIS] planner_time={metrics_msg.planner_time_ms}ms, "
            f"actions={action_count}"
        )
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Feedback: {feedback.status} - pos=({feedback.current_x:.2f}, "
            f"{feedback.current_y:.2f}, {feedback.current_z:.2f}) "
            f"dist={feedback.distance_remaining:.2f}m"
        )
    
    def _execute_navigate_to(self, target_x: float, target_y: float, target_z: float, standoff: float = 2.0) -> tuple:
        """
        Execute a navigate_to action using the path planner.
        
        This method:
        1. Calls the path planner service to get waypoints around obstacles
        2. Executes goto for each waypoint
        3. Stops at a standoff distance from the target (unless standoff=0)
        
        Note: Coordinates come directly from the intent context (enriched by LGA/EKG),
        so we don't need to query EKG again here!
        
        Args:
            target_x: Target X coordinate in meters
            target_y: Target Y coordinate in meters  
            target_z: Target Z coordinate (flight height)
            standoff: Distance to keep from target (default 2.0m)
        
        Returns:
            tuple: (success: bool, message: str)
        """
        import time
        import math
        
        self.get_logger().info(f"===== Navigate To: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) [standoff={standoff}] =====")
        
        # -------------------------------------------------------------
        # STANDOFF DISTANCE: Stop well away from the target object
        # For large objects (like 3x3m pallets), we need to stay outside
        # the object footprint. 2.0m standoff should be safe for most cases.
        # -------------------------------------------------------------
        standoff_distance = float(standoff)  # use param
        
        # Calculate direction from current position to target
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance_to_target = math.sqrt(dx * dx + dy * dy)
        
        # Only apply standoff if we're far enough away
        if distance_to_target > standoff_distance:
            # Shorten the target by standoff distance
            scale = (distance_to_target - standoff_distance) / distance_to_target
            adjusted_x = self.position[0] + dx * scale
            adjusted_y = self.position[1] + dy * scale
            self.get_logger().info(
                f"  Applying {standoff_distance}m standoff: "
                f"({target_x:.2f}, {target_y:.2f}) → ({adjusted_x:.2f}, {adjusted_y:.2f})"
            )
            target_x = adjusted_x
            target_y = adjusted_y
        else:
            # We are already within the standoff distance (or closer)
            # Do NOT try to fly to the center of the object!
            self.get_logger().info(
                f"  Already within standoff distance ({distance_to_target:.2f}m <= {standoff_distance}m). "
                f"Holding position to avoid collision."
            )
            return True, f"Already near target ({distance_to_target:.2f}m)"
        
        # -------------------------------------------------------------
        # STEP 1: Call path planner to get waypoints
        # -------------------------------------------------------------
        
        self.get_logger().info("Calling path planner...")
        
        if not self._path_planner_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Path planner not available, using direct goto")
            # Fallback: go directly (no obstacle avoidance)
            return self._execute_goto(target_x, target_y, target_z)
        
        # Build path planning request
        plan_request = PlanPath.Request()
        plan_request.start = Point(
            x=float(self.position[0]),
            y=float(self.position[1]),
            z=float(self.position[2])
        )
        plan_request.goal = Point(
            x=float(target_x),
            y=float(target_y),
            z=float(target_z)
        )
        plan_request.flight_height = float(target_z)
        plan_request.safety_margin = 0.8  # 80cm safety margin for reliable clearance
        self.get_logger().info(f"  Safety margin: {plan_request.safety_margin}m")
        
        plan_future = self._path_planner_client.call_async(plan_request)
        
        # Wait for path planning result
        start_time = time.time()
        while not plan_future.done():
            if time.time() - start_time > 10.0:  # 10 second timeout
                return False, "Path planning timed out"
            time.sleep(0.05)
        
        plan_result = plan_future.result()
        
        if not plan_result.success:
            return False, f"Path planning failed: {plan_result.message}"
        
        waypoints = plan_result.waypoints
        self.get_logger().info(
            f"  Path found! {len(waypoints)} waypoints, "
            f"{plan_result.total_distance:.2f}m total"
        )
        
        # -------------------------------------------------------------
        # STEP 2: Navigate through each waypoint
        # -------------------------------------------------------------
        
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(
                f"  [{i+1}/{len(waypoints)}] Going to "
                f"({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})"
            )
            
            success, message = self._execute_goto(
                waypoint.x, waypoint.y, waypoint.z
            )
            
            if not success:
                return False, f"Failed at waypoint {i+1}: {message}"
        
        self.get_logger().info(f"===== Navigation to ({target_x:.2f}, {target_y:.2f}) complete! =====")
        return True, f"Navigated to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
    
    def _execute_goto(self, x: float, y: float, z: float) -> tuple:
        """
        Execute a single goto action.
        
        Returns:
            tuple: (success: bool, message: str)
        """
        import time
        
        # Build the goal
        goal = DroneCommand.Goal()
        goal.command_type = 'goto'
        goal.target_x = float(x)
        goal.target_y = float(y)
        goal.target_z = float(z)
        goal.yaw = 0.0
        goal.duration = 2.0
        
        # Send goal
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        
        while not send_future.done():
            time.sleep(0.01)
        
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            return False, "Goal rejected"
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.01)
        
        result = result_future.result().result
        
        if result.success:
            # Update our internal position state
            self.position[0] = x
            self.position[1] = y
            self.position[2] = z
            return True, result.message
        else:
            return False, result.message

    
    def _format_intent_for_prompt(self, num: int, intent, context: str) -> str:
        """Format an intent with clear action hints for the LLM planner"""
        intent_type = intent.intent_type
        params = {p.split(':')[0]: p.split(':')[1] if ':' in p else '' 
                  for p in intent.parameters}
        
        if intent_type == "CHANGE_ALTITUDE":
            direction = params.get('direction', 'up')
            distance = params.get('distance', '0')
            desc = f"{num}. CHANGE_ALTITUDE: Move {direction} by {distance} meters"
            desc += f" → use goto with z = current_z + {distance}" if direction == 'up' else f" → use goto with z = current_z - {distance}"
            if context:
                desc += f" [context: {context}]"
        
        elif intent_type == "MOVE_DIRECTION":
            direction = params.get('direction', 'forward').lower()
            try:
                distance = float(params.get('distance', '1.0'))
            except ValueError:
                distance = 1.0
            
            # Parse current position from context to compute target
            import re
            pos_match = re.search(r'drone_pos:\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)', context) if context else None
            if pos_match:
                curr_x, curr_y, curr_z = float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3))
            else:
                # Fallback to node's cached position
                curr_x, curr_y, curr_z = self.position[0], self.position[1], self.position[2]
            
            # Calculate target based on direction (matching LGA: forward=+X, left=+Y)
            # Use minimum 1.0m flight height - if grounded (z < 1.0), takeoff will happen first
            target_z = max(curr_z, 1.0)
            target_x, target_y = curr_x, curr_y
            if direction == 'forward':
                target_x = curr_x + distance  # +X direction
            elif direction == 'backward':
                target_x = curr_x - distance  # -X direction
            elif direction == 'left':
                target_y = curr_y + distance  # +Y direction
            elif direction == 'right':
                target_y = curr_y - distance  # -Y direction
            
            desc = f"{num}. MOVE_DIRECTION: Move {direction} by {distance}m"
            desc += f" → EXACT goto params: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}"
            # CRITICAL: Include full context with safety info (obstacles, ACTION NOT SAFE, etc.)
            if context:
                desc += f"\n   [SAFETY CONTEXT: {context}]"
        
        elif intent_type == "GO_TO_LOCATION":
            target = params.get('target', 'unknown')
            desc = f"{num}. GO_TO_LOCATION: Navigate to {target}"
            # Always include context if available - it contains target coordinates
            # Context may contain "shelf_a at (-8.00, 6.00, 1.00)" OR "target_coords: (0.0, 0.0, 0.0)"
            if context:
                # SPECIAL HANDLING: For GO_TO, don't show drone_pos in prompt to avoid confusion
                # Context usually looks like: "drone_pos: (...); is_flying: ...; shelf_b at (x,y)"
                # We only want the target part for the PROMPT (planner parses state separately)
                
                import re
                # Check for explicit target format "name at (x, y)"
                match_target = re.search(r'(\w+)\s+at\s+\(([^)]+)\)', context)
                # Check for coordinates format "target_coords: (x, y, z)"
                match_coords = re.search(r'target_coords:\s*\(([^)]+)\)', context)
                
                if match_target:
                    # Found named target, e.g. "shelf_b at (10.0, 8.0)"
                    # Also look for flight_height if present
                    height_part = ""
                    match_height = re.search(r'flight_height:\s*([\d.]+)', context)
                    if match_height:
                        height_part = f"; flight_height: {match_height.group(1)}"
                        
                    desc += f" → TARGET: {match_target.group(1)} at ({match_target.group(2)}){height_part}"
                    
                elif match_coords:
                    # Found direct coords (from "go back" etc)
                    desc += f" → TARGET COORDS: ({match_coords.group(1)})"
                    
                else:
                    # Fallback: if we can't extract clean target, show full context 
                    # but try to strip common state prefixes to reduce confusion
                    clean_ctx = context
                    for prefix in ["drone_pos", "is_flying", "drone_state", "battery"]:
                        clean_ctx = re.sub(f"{prefix}:[^;]+;?\s*", "", clean_ctx)
                    desc += f" → {clean_ctx}"
        
        elif intent_type == "HOVER":
            duration = params.get('duration', '2.0')
            desc = f"{num}. HOVER: Stay in place for {duration}s → use hover tool"
            if context:
                desc += f" [context: {context}]"
        
        elif intent_type == "TAKEOFF":
            height = params.get('height', '1.0')
            desc = f"{num}. TAKEOFF: Take off to {height}m → use takeoff tool"
            if context:
                desc += f" [context: {context}]"
        
        elif intent_type == "LAND":
            desc = f"{num}. LAND: Land the drone → use land tool"
            if context:
                desc += f" [context: {context}]"
        
        elif intent_type == "ROTATE":
            direction = params.get('direction', 'right').lower()
            angle_str = params.get('angle', '90')
            # remove 'degrees' if present
            angle_str = angle_str.replace('degrees', '').replace('deg', '').strip()
            try:
                angle = float(angle_str)
            except ValueError:
                angle = 90.0
            
            # Calculate target yaw
            current_yaw = self.yaw
            target_yaw = current_yaw
            
            if direction == 'left':
                target_yaw += angle
            else: # right
                target_yaw -= angle
                
            # Normalize to -180..180
            while target_yaw > 180: target_yaw -= 360
            while target_yaw < -180: target_yaw += 360
            
            desc = f"{num}. ROTATE: Turn {direction} by {angle} degrees"
            desc += f" → CURRENT YAW = {current_yaw:.1f}. Target yaw = {target_yaw:.1f}."
            desc += f" Use goto tool with yaw={target_yaw:.1f}, x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}."
            if context:
                desc += f" [context: {context}]"

        else:
            params_str = ", ".join(intent.parameters) if intent.parameters else "none"
            desc = f"{num}. {intent_type} (params: {params_str})"
            if context:
                desc += f" [context: {context}]"
        

        
        return desc
    
    def _handle_query_intent(self, query_intent, raw_command: str):
        """
        Handle a QUERY intent by generating a response from context.
        
        Instead of sending to the action server, we use the LLM to generate
        a natural language response based on the drone's current state.
        
        Args:
            query_intent: The SingleIntent with intent_type='QUERY'
            raw_command: The original user command
        """
        # Extract query type from parameters
        params = {p.split(':')[0]: p.split(':')[1] if ':' in p else '' 
                  for p in query_intent.parameters}
        query_type = params.get('query_type', 'status').lower()
        context = query_intent.context if query_intent.context else ""
        
        self.get_logger().info(f"Handling QUERY intent: query_type={query_type}")
        self.get_logger().info(f"  Context: {context[:100]}...")
        
        # Build a response based on query type and context
        response = self._generate_query_response(query_type, context, raw_command)
        
        # Publish the response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        
        self.get_logger().info(f"Published query response: {response}")
        
        # Also publish as execution feedback for logging
        feedback = ExecutionFeedback()
        feedback.command = raw_command
        feedback.intent_type = "QUERY"
        feedback.success = True
        feedback.actions = ["query"]
        feedback.results = [response]
        feedback.summary = f"Query answered: {query_type}"
        feedback.stamp = self.get_clock().now().to_msg()
        self.feedback_pub.publish(feedback)
    
    def _generate_query_response(self, query_type: str, context: str, raw_command: str) -> str:
        """
        Generate a natural language response for a query using the LLM.
        
        Uses the LLM to create a conversational response based on the
        drone's current state from the LGA context.
        
        Args:
            query_type: Type of query (battery, location, altitude, status)
            context: The LGA context string with current state
            raw_command: Original user command
            
        Returns:
            Natural language response string
        """
        # Always use the LLM for natural, conversational responses
        return self._llm_generate_response(context, raw_command)
    
    def _llm_generate_response(self, context: str, raw_command: str) -> str:
        """
        Use the LLM to generate a natural language response for complex queries.
        
        Args:
            context: The LGA context string with current state
            raw_command: Original user command
            
        Returns:
            Natural language response string
        """
        prompt = f"""You are a helpful drone assistant. Answer the user's question based on the current drone state.

Current Drone State:
{context}

User Question: {raw_command}

Provide a brief, natural response (1-2 sentences). Be conversational but informative.

Response:"""
        
        try:
            response = requests.post(
                f"{self.llm_server_url}/v1/completions",
                json={
                    "prompt": prompt,
                    "max_tokens": 100,
                    "temperature": 0.3,
                    "stop": ["\n\n", "User:", "<|im_end|>"],
                },
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                return result["choices"][0]["text"].strip()
            else:
                return f"I'm at position ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})."
        except Exception as e:
            self.get_logger().warn(f"LLM response generation failed: {e}")
            return f"I'm currently {'flying' if self.is_flying else 'grounded'} at ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})."
    
    def _extract_context(self, parameters: list) -> str:
        """Extract context string from intent parameters"""
        for param in parameters:
            if param.startswith("context:"):
                return param[8:]  # Remove "context:" prefix
        return ""
    
    def _generate_plan_for_intent(self, single_intent, raw_command: str, context: str) -> list:
        """Generate action plan for a single intent using LLM"""
        # Build prompt using SingleIntent fields
        prompt = self.prompt_template.format(
            is_flying=str(self.is_flying).lower(),
            pos_x=self.position[0],
            pos_y=self.position[1],
            pos_z=self.position[2],
            tools_prompt=get_tools_prompt(),
            intent_type=single_intent.intent_type,
            raw_command=raw_command,
            context=context if context else "No additional context"
        )
        
        self.get_logger().debug(f"Prompt:\n{prompt}")
        
        try:
            # Use HTTP server
            self.get_logger().info(f"Sending prompt for {single_intent.intent_type}...")
            
            response = requests.post(
                f"{self.llm_server_url}/v1/completions",
                json={
                    "prompt": prompt,
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "stop": ["\n\n\n", "User:", "<|im_end|>"],
                },
                timeout=120
            )
            
            if response.status_code != 200:
                self.get_logger().error(f"HTTP error: {response.status_code}")
                return []
            
            result = response.json()
            raw_text = result["choices"][0]["text"].strip()
            
            self.get_logger().info(f"LLM response: {raw_text[:200]}...")
            
            return self._parse_plan(raw_text)
            
        except requests.exceptions.Timeout:
            self.get_logger().error("LLM request timed out")
            return []
        except Exception as e:
            self.get_logger().error(f"LLM inference failed: {e}")
            return []
    
    def _generate_plan(self, intent: Intent, context: str) -> list:
        """Generate action plan using LLM (legacy - for backward compat)"""
        # Build prompt
        prompt = self.prompt_template.format(
            is_flying=str(self.is_flying).lower(),
            pos_x=self.position[0],
            pos_y=self.position[1],
            pos_z=self.position[2],
            tools_prompt=get_tools_prompt(),
            intent_type=intent.intent_type if hasattr(intent, 'intent_type') else "UNKNOWN",
            raw_command=intent.raw_command,
            context=context if context else "No additional context"
        )
        
        self.get_logger().debug(f"Prompt:\n{prompt}")
        
        try:
            # Use HTTP server instead of llama_ros
            self.get_logger().info("Sending prompt to llama.cpp HTTP server...")
            
            response = requests.post(
                f"{self.llm_server_url}/v1/completions",
                json={
                    "prompt": prompt,
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "stop": ["\n\n\n", "User:", "<|im_end|>"],
                },
                timeout=120
            )
            
            if response.status_code != 200:
                self.get_logger().error(f"HTTP error: {response.status_code}")
                return []
            
            result = response.json()
            raw_text = result["choices"][0]["text"].strip()
            
            self.get_logger().info(f"LLM response: {raw_text[:200]}...")
            
            # Parse JSON response
            return self._parse_plan(raw_text)
            
        except requests.exceptions.Timeout:
            self.get_logger().error("LLM request timed out")
            return []
        except Exception as e:
            self.get_logger().error(f"LLM inference failed: {e}")
            return []
    
    def _parse_plan(self, raw_text: str) -> list:
        """Parse LLM response into action list"""
        try:
            # Clean up response
            text = raw_text.strip()
            
            # Handle markdown code blocks
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0]
            elif "```" in text:
                text = text.split("```")[1].split("```")[0]
            text = text.strip()
            
            # Try to extract and parse JSON objects - try multiple if first fails
            remaining_text = text
            attempt = 0
            max_attempts = 3  # Try up to 3 JSON objects
            
            while attempt < max_attempts and remaining_text:
                json_str = self._extract_first_json_object(remaining_text)
                if not json_str:
                    break
                
                try:
                    data = json.loads(json_str)
                    reasoning = data.get("reasoning", "")
                    actions = data.get("actions", [])
                    
                    # FALLBACK: Handle case where LLM returns bare action object
                    # e.g., {"tool": "goto", "params": {...}} instead of {"reasoning": ..., "actions": [...]}
                    if not actions and "tool" in data:
                        self.get_logger().warn("LLM returned bare action object, wrapping in actions array")
                        actions = [data]
                        reasoning = "Bare action (no wrapper)"
                    
                    if attempt > 0:
                        self.get_logger().info(f"Used JSON object #{attempt+1} (earlier ones had parse errors)")
                    
                    self.get_logger().info(f"LLM reasoning: {reasoning}")
                    return actions
                    
                except json.JSONDecodeError as e:
                    self.get_logger().warn(f"JSON object #{attempt+1} parse failed: {e}")
                    # Move past this JSON object and try the next one
                    end_pos = remaining_text.find(json_str) + len(json_str)
                    remaining_text = remaining_text[end_pos:].strip()
                    attempt += 1
            
            self.get_logger().error("No valid JSON object found in response")
            self.get_logger().error(f"Raw text: {raw_text}")
            return []
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse LLM response: {e}")
            self.get_logger().error(f"Raw text: {raw_text}")
            return []
    
    def _extract_first_json_object(self, text: str) -> str:
        """
        Extract the first complete JSON object from text using bracket matching.
        This handles cases where LLM outputs multiple JSON objects.
        """
        # Find the first opening brace
        start = text.find('{')
        if start == -1:
            return ""
        
        # Count braces to find the matching closing brace
        brace_count = 0
        for i, char in enumerate(text[start:], start):
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    # Found the matching closing brace
                    return text[start:i+1]
        
        # No matching closing brace found
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
