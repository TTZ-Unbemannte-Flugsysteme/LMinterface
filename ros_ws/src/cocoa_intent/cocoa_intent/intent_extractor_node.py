"""
Intent Extractor Node for Cocoa Speech
LLM-based intent classification and parameter extraction
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from cocoa_msgs.msg import Intent, SingleIntent, ExecutionFeedback, UserFeedback, ThesisMetrics

from .llama_http_interface import LlamaHTTPInterface, IntentResult
from .openai_interface import OpenAIInterface, OPENAI_AVAILABLE
from cocoa_msgs.srv import QueryLGA, GetKnowledgeSummary
from geometry_msgs.msg import Point, PoseStamped


class IntentExtractorNode(Node):
    """
    Intent Extractor ROS2 Node
    
    Subscribes to text input, uses LLM to classify intent,
    publishes structured Intent messages.
    """
    
    def __init__(self):
        super().__init__('intent_extractor_node')
        
        # Create callback group for service calls
        self.service_cb_group = ReentrantCallbackGroup()
        
        # Declare parameters (using HTTP server now instead of llama_ros)
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('max_tokens', 512)
        self.declare_parameter('input_topic', '/voice/text')
        self.declare_parameter('output_topic', '/intent')
        self.declare_parameter('llm_server_url', 'http://localhost:8081')
        self.declare_parameter('assume_ambiguous', True)  # If True, LLM picks best match for ambiguous refs
        self.declare_parameter('drone_id', 'cf231')  # Drone ID for pose subscription
        self.declare_parameter('use_cot', True)  # Enable Chain-of-Thought reasoning
        
        # THESIS PARAMETER: Toggle between LGA filtering and baseline (full context)
        self.declare_parameter('use_lga', False)
        self.use_lga = self.get_parameter('use_lga').value
        
        # LLM Backend selection: 'llama' (local) or 'openai' (API)
        self.declare_parameter('llm_backend', 'llama')  # 'llama' or 'openai'
        self.declare_parameter('openai_model', 'gpt-4o-mini')  # gpt-4o-mini, gpt-4o, gpt-4-turbo
        
        # Get parameters
        temperature = self.get_parameter('temperature').value
        max_tokens = self.get_parameter('max_tokens').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        server_url = self.get_parameter('llm_server_url').value
        self.assume_ambiguous = self.get_parameter('assume_ambiguous').get_parameter_value().bool_value
        use_cot_init = self.get_parameter('use_cot').get_parameter_value().bool_value
        
        self.get_logger().info(f"Assume ambiguous: {self.assume_ambiguous}")
        self.get_logger().info(f"Chain-of-Thought enabled: {use_cot_init}")
        
        # Load system prompt
        system_prompt = self._load_system_prompt()
        
        # Initialize LLM based on backend selection
        llm_backend = self.get_parameter('llm_backend').value
        
        if llm_backend == 'openai':
            if not OPENAI_AVAILABLE:
                self.get_logger().error("OpenAI backend selected but 'openai' package not installed!")
                self.get_logger().error("Install with: pip install openai")
                raise ImportError("openai package required for llm_backend='openai'")
            
            openai_model = self.get_parameter('openai_model').value
            self.get_logger().info(f"Using OpenAI backend with model: {openai_model}")
            self.llm = OpenAIInterface(
                node=self,
                system_prompt=system_prompt,
                model=openai_model,
                temperature=temperature,
                max_tokens=max_tokens,
            )
        else:
            # Default: llama.cpp HTTP server
            self.get_logger().info("Using llama.cpp HTTP backend for intent extraction...")
            self.llm = LlamaHTTPInterface(
                node=self,
                system_prompt=system_prompt,
                server_url=server_url,
                temperature=temperature,
                max_tokens=max_tokens,
            )
        
        # Publishers and subscribers
        self.intent_pub = self.create_publisher(Intent, output_topic, 10)
        self.text_sub = self.create_subscription(
            String, input_topic, self._text_callback, 10,
            callback_group=self.service_cb_group
        )
        
        # Create LGA service client (single gateway for all context)
        self.lga_client = self.create_client(
            QueryLGA, '/lga/query',
            callback_group=self.service_cb_group
        )
        
        # Create EKG knowledge summary client (for LLM prompt injection)
        self.ekg_summary_client = self.create_client(
            GetKnowledgeSummary, '/ekg/get_knowledge_summary',
            callback_group=self.service_cb_group
        )
        
        # Fetch static EKG knowledge at startup (one-time)
        self.ekg_knowledge = self._fetch_ekg_knowledge()
        
        # Store current drone position (updated via pose callback)
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        
        # Subscribe to drone pose for accurate position tracking
        self.drone_id = self.get_parameter('drone_id').value
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_id}/pose',
            self._pose_callback,
            10,
            callback_group=self.service_cb_group
        )
        self.get_logger().info(f"Subscribing to pose: /{self.drone_id}/pose")
        
        # Conversation history (last 3 exchanges) - kept small to avoid exceeding LLM context
        self.conversation_history = []
        self.max_history = 3
        
        # Service to reset conversation history (used by test runner between sequences)
        self.reset_history_srv = self.create_service(
            Trigger, '/intent_extractor/reset_history',
            self._reset_history_callback,
            callback_group=self.service_cb_group
        )
        
        # THESIS METRICS: Track token usage for A/B comparison
        self.metrics = {
            'total_requests': 0,
            'lga_requests': 0,
            'baseline_requests': 0,
            'total_context_tokens': 0,
            'successes': 0,
            'failures': 0,
        }
        
        # Subscribe to execution feedback from planner
        self.feedback_sub = self.create_subscription(
            ExecutionFeedback,
            '/execution_feedback',
            self._execution_feedback_callback,
            10,
            callback_group=self.service_cb_group
        )
        
        # Publisher for user feedback (rejections, clarifications)
        self.user_feedback_pub = self.create_publisher(
            UserFeedback,
            '/user_feedback',
            10
        )
        
        # THESIS: Publisher for experiment metrics
        self.metrics_pub = self.create_publisher(
            ThesisMetrics,
            '/thesis_metrics',
            10
        )
        
        # Confidence threshold for rejection
        self.confidence_threshold = 0.5
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Intent Extractor Ready")
        self.get_logger().info(f"  Input: {input_topic}")
        self.get_logger().info(f"  Output: {output_topic}")
        # THESIS: Show context mode
        context_mode = "LGA FILTERED" if self.use_lga else "BASELINE (full EKG)"
        self.get_logger().info(f"  Context mode: {context_mode}")
        self.get_logger().info(f"  EKG knowledge: {len(self.ekg_knowledge)} chars")
        self.get_logger().info(f"  History: last {self.max_history} exchanges")
        self.get_logger().info(f"  Rejection threshold: confidence < {self.confidence_threshold}")
        mode = "ASSUME (pick best match)" if self.assume_ambiguous else "STRICT (reject ambiguous)"
        self.get_logger().info(f"  Disambiguation mode: {mode}")
        self.get_logger().info("=" * 50)

    def _pose_callback(self, msg: PoseStamped):
        """Update current drone position from pose messages"""
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z

    def _load_system_prompt(self) -> str:
        """Load system prompt from file"""
        try:
            pkg_share = get_package_share_directory('cocoa_intent')
            prompt_path = os.path.join(pkg_share, 'prompts', 'intent_extraction.txt')
            with open(prompt_path, 'r') as f:
                return f.read()
        except Exception as e:
            self.get_logger().warn(f"Could not load prompt file: {e}, using default")
            return self._get_default_prompt()

    def _get_default_prompt(self) -> str:
        """Default prompt if file not found"""
        return """You are an intent classifier for a drone control system.
Classify the user command into one intent type and extract parameters.

INTENT TYPES: MOVE_DIRECTION, GO_TO_LOCATION, CHANGE_ALTITUDE, ROTATE, 
TAKEOFF, LAND, HOVER, EMERGENCY_STOP, QUERY, UNKNOWN

Output JSON only:
{"intent_type": "TYPE", "parameters": {}, "confidence": 0.95}

Now classify this command:
"""

    def _fetch_ekg_knowledge(self) -> str:
        """Fetch static environment knowledge from EKG at startup.
        
        This is called ONCE at initialization, not per-query.
        Returns serialized knowledge graph for LLM prompt injection (~60 tokens).
        Includes retry logic in case EKG node starts after Intent Extractor.
        
        Uses spin_until_future_complete since node isn't spinning yet during __init__.
        """
        import rclpy
        import time
        
        max_retries = 3
        retry_delay = 2.0
        
        for attempt in range(max_retries):
            # Wait for EKG service to be available
            self.get_logger().info(f"Waiting for EKG service (attempt {attempt + 1}/{max_retries})...")
            if not self.ekg_summary_client.wait_for_service(timeout_sec=10.0):
                if attempt < max_retries - 1:
                    self.get_logger().warn(f"EKG service not available, retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    continue
                else:
                    self.get_logger().error("EKG knowledge summary service not available after all retries")
                    return ""
            
            try:
                request = GetKnowledgeSummary.Request()
                future = self.ekg_summary_client.call_async(request)
                
                # Use spin_until_future_complete since we're in __init__ and not spinning yet
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if not future.done():
                    self.get_logger().error("EKG knowledge fetch timed out")
                    if attempt < max_retries - 1:
                        continue
                    return ""
                
                result = future.result()
                if result and result.knowledge_text:
                    self.get_logger().info(
                        f"Loaded EKG knowledge: {result.object_count} objects, "
                        f"{result.relationship_count} relationships"
                    )
                    return result.knowledge_text
                return ""
                
            except Exception as e:
                self.get_logger().error(f"Failed to fetch EKG knowledge: {e}")
                if attempt < max_retries - 1:
                    continue
                return ""
        
        return ""

    def _reset_history_callback(self, request, response):
        """Service handler to clear conversation history (used between test sequences)."""
        count = len(self.conversation_history)
        self.conversation_history = []
        self.get_logger().info(f'[RESET] Cleared {count} conversation history entries')
        response.success = True
        response.message = f'Cleared {count} history entries'
        return response
    
    def _format_conversation_history(self, include_raw_commands: bool = False) -> str:
        """Format conversation history for inclusion in prompt.
        
        Improved format includes position data for ALL commands,
        enabling proper "go back" reasoning even after relative movements.
        """
        if not self.conversation_history:
            return ""
        
        # New format: include from/to positions for every action
        # This enables the LLM to understand spatial context for "go back"
        history_lines = ["\nHistory:"]
        
        previous_positions = []  # Track all positions for "go back"
        
        for i, exchange in enumerate(self.conversation_history, 1):
            status = "OK" if exchange.get('execution_status') == 'SUCCESS' else "FAIL"
            intent_type = exchange.get('intent_type', 'UNKNOWN')
            params = exchange.get('parameters', {})
            
            # Get key param (target or direction)
            target = params.get('target', '')
            direction = params.get('direction', '')
            distance = params.get('distance', '')
            param_str = f"→{target}" if target else (f"→{direction}" if direction else "")
            
            # Get position data - this was stored at command time
            pos = exchange.get('drone_position', (0.0, 0.0, 0.0))
            pos_str = f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
            
            # Track positions for "go back" - the position BEFORE this command
            previous_positions.append(pos)
            
            # Build history line with position context
            raw_cmd = exchange.get('command', '')
            cmd_prefix = f'"{raw_cmd}" → ' if (include_raw_commands and raw_cmd) else ''
            if intent_type == 'MOVE_DIRECTION' and distance:
                history_lines.append(f"[{i}] {cmd_prefix}{intent_type}{param_str} {distance}m | from:{pos_str} [{status}]")
            elif intent_type == 'GO_TO_LOCATION':
                history_lines.append(f"[{i}] {cmd_prefix}{intent_type}{param_str} | from:{pos_str} [{status}]")
            elif intent_type == 'CHANGE_ALTITUDE':
                alt_dir = params.get('direction', 'up')
                alt_dist = params.get('distance', '')
                history_lines.append(f"[{i}] {cmd_prefix}{intent_type}→{alt_dir} {alt_dist}m | from:{pos_str} [{status}]")
            else:
                history_lines.append(f"[{i}] {cmd_prefix}{intent_type}{param_str} | from:{pos_str} [{status}]")
        
        # Add current position (where drone IS now)
        curr_pos = (self.current_position.x, self.current_position.y, self.current_position.z)
        history_lines.append(f"CURRENT_POS: ({curr_pos[0]:.1f}, {curr_pos[1]:.1f}, {curr_pos[2]:.1f})")
        
        # Add previous position for "go back" - the position before the LAST command
        if previous_positions:
            prev_pos = previous_positions[-1]
            history_lines.append(f"PREV_POS: ({prev_pos[0]:.1f}, {prev_pos[1]:.1f}, {prev_pos[2]:.1f}) ← for 'go back'")
        
        # Also include named location context if available
        successful_targets = []
        for exchange in self.conversation_history:
            if (exchange.get('execution_status') == 'SUCCESS' and 
                exchange.get('intent_type') == 'GO_TO_LOCATION'):
                target = exchange.get('parameters', {}).get('target', '')
                if target and not target.startswith('('):  # Skip coordinate targets
                    successful_targets.append(target)
        
        if successful_targets:
            history_lines.append(f"LAST_NAMED_LOC: {successful_targets[-1]}")
            if len(successful_targets) >= 2:
                history_lines.append(f"PREV_NAMED_LOC: {successful_targets[-2]}")
        
        # Explicitly label the last action for "do that again" commands
        if self.conversation_history:
            last = self.conversation_history[-1]
            params = last.get('parameters', {})
            target = params.get('target', '')
            direction = params.get('direction', '')
            p_str = f"→{target}" if target else (f"→{direction}" if direction else "")
            history_lines.append(f"LAST_ACTION: {last['intent_type']}{p_str}")

        return "\n".join(history_lines)
    
    def _add_to_history(self, text: str, result, execution_status: str = 'PENDING'):
        """Add an exchange to conversation history, keeping only last max_history entries
        
        Args:
            text: The user's command
            result: IntentResult from LLM
            execution_status: Initial status - 'PENDING' for normal commands, 
                              'REJECTED' for rejected commands
        """
        entry = {
            'command': text,
            'intent_type': result.intent_type,
            'parameters': result.parameters,
            'confidence': result.confidence,
            'execution_status': execution_status,
            'action_details': [],  # Will store per-action results for executed commands
            # Store a COPY of position (not reference) - using tuple to avoid mutation
            'drone_position': (self.current_position.x, self.current_position.y, self.current_position.z)
        }
        
        self.conversation_history.append(entry)
        
        # Keep only the last max_history entries
        if len(self.conversation_history) > self.max_history:
            self.conversation_history = self.conversation_history[-self.max_history:]
    
    def _execution_feedback_callback(self, msg: ExecutionFeedback):
        """Process execution feedback and update conversation history"""
        # Find the matching history entry by command text
        for entry in reversed(self.conversation_history):
            if entry['command'] == msg.command and entry['execution_status'] == 'PENDING':
                # Update overall status
                entry['execution_status'] = 'SUCCESS' if msg.success else 'FAILED'
                
                # Build detailed action results
                action_details = []
                for i, (action, result) in enumerate(zip(msg.actions, msg.results)):
                    if 'SUCCESS' in result:
                        action_details.append(f"{action}=OK")
                    elif 'FAILED' in result:
                        # Extract failure reason if present
                        reason = result.replace('FAILED:', '').replace('FAILED', '').strip()
                        if reason:
                            action_details.append(f"{action}=FAILED({reason[:20]})")
                        else:
                            action_details.append(f"{action}=FAILED")
                    elif 'REJECTED' in result:
                        action_details.append(f"{action}=REJECTED")
                    else:
                        action_details.append(f"{action}={result[:15]}")
                
                entry['action_details'] = action_details
                
                self.get_logger().info(
                    f"Updated history: '{msg.command}' -> {entry['execution_status']} | {action_details}"
                )
                break
    
    def _text_callback(self, msg: String):
        """Process incoming text and extract intent(s)"""
        text = msg.data.strip()
        if not text:
            return
        
        self.get_logger().info(f'Received text: "{text}"')
        
        # Get conversation history for context
        history = self._format_conversation_history()
        
        # Debug: Print conversation history
        if history:
            self.get_logger().info(f'[DEBUG] Conversation history:\n{history}')
        else:
            self.get_logger().info('[DEBUG] Conversation history: (empty)')
        
        # Extract intent(s) using LLM with EKG knowledge for disambiguation
        # Include drone position so LLM can resolve "near me" type queries
        drone_pos = (self.current_position.x, self.current_position.y, self.current_position.z)
        
        # Check if CoT is enabled (dynamic parameter)
        use_cot = self.get_parameter('use_cot').get_parameter_value().bool_value
        
        intent_results = self.llm.extract_intent(
            text, 
            conversation_history=history,
            environment_knowledge=self.ekg_knowledge,
            assume_ambiguous=self.assume_ambiguous,
            drone_position=drone_pos,
            use_cot=use_cot
        )
        
        if not intent_results:
            self.get_logger().error("Intent extraction failed")
            self._publish_user_feedback(
                text, "REJECTION", 
                "I couldn't understand that command. Please try rephrasing.",
                0.0
            )
            return
            
        # Ensure all parameters are strings to avoid ROS message type assertion errors
        for res in intent_results:
            if hasattr(res, 'parameters') and isinstance(res.parameters, dict):
                for k in list(res.parameters.keys()):
                    v = res.parameters[k]
                    if isinstance(v, list):
                        res.parameters[k] = ", ".join(str(item) for item in v)
                    elif not isinstance(v, str):
                        res.parameters[k] = str(v)
        
        
        self.get_logger().info(f'Extracted {len(intent_results)} intent(s)')
        
        # Build Intent message with array of SingleIntents
        intent_msg = Intent()
        intent_msg.raw_command = text
        intent_msg.stamp = self.get_clock().now().to_msg()
        
        # Include conversation history only for COMPLEX_TASK intents
        # (simple intents are fully resolved by the extractor, but complex tasks
        # may need history for referential commands like "do that again")
        has_complex = any(r.intent_type == "COMPLEX_TASK" for r in intent_results)
        if has_complex:
            # Re-format history with raw commands included for the planner
            planner_history = self._format_conversation_history(include_raw_commands=True)
            if planner_history:
                intent_msg.conversation_history = planner_history
        
        # Process each extracted intent
        all_rejected = True
        for result in intent_results:
            self.get_logger().info(
                f'Intent: {result.intent_type} | '
                f'Params: {result.parameters} | '
                f'Confidence: {result.confidence:.2f}'
            )
            
            # === REJECTION LOGIC ===
            rejection_reason = self._should_reject_intent(result, text)
            if rejection_reason:
                self.get_logger().warn(f'Rejecting intent {result.intent_type}: {rejection_reason}')
                continue  # Skip this intent but process others
            
            all_rejected = False
            
            # Get context from LGA for this specific intent
            context = self._get_context_for_intent(result)
            if context:
                self.get_logger().info(f'Context for {result.intent_type}: {context}')
            
            # Build SingleIntent message
            # Note: Context includes safety info ("ACTION NOT SAFE") - Planner LLM should check this
            single_intent = SingleIntent()
            single_intent.intent_type = result.intent_type
            single_intent.parameters = [f"{k}:{v}" for k, v in result.parameters.items()]
            single_intent.confidence = result.confidence
            single_intent.context = context if context else ""
            
            intent_msg.intents.append(single_intent)
        
        # If all intents were rejected, publish error feedback
        if all_rejected:
            self.get_logger().warn('All intents rejected')
            self._publish_user_feedback(
                text, "REJECTION",
                "I cannot perform this action. Please try a different command.",
                intent_results[0].confidence
            )
            self._add_to_history(text, intent_results[0], execution_status='REJECTED')
            return
        
        # Add to conversation history (use first valid intent for history)
        self._add_to_history(text, intent_results[0])
        
        # Publish the Intent message with all valid intents
        self.intent_pub.publish(intent_msg)
        intent_types = [si.intent_type for si in intent_msg.intents]
        self.get_logger().info(f'Published {len(intent_msg.intents)} intent(s): {intent_types}')
        
        # THESIS: Publish metrics for test runner
        self._publish_thesis_metrics(text, intent_results)
    
    def _should_reject_intent(self, result, text: str) -> str:
        """
        Check if intent should be rejected. Returns rejection reason or empty string.
        """
        # Reject UNKNOWN intents
        if result.intent_type == "UNKNOWN":
            return f"I don't understand the command '{text}'. Please specify a valid drone action like takeoff, land, move, or go to a location."
            
        # COMPLEX_TASK is valid - pass it through to the strategic planner
        if result.intent_type == "COMPLEX_TASK":
            return ""
        
        # Reject GO_TO_LOCATION with missing, empty, or UNKNOWN target
        if result.intent_type == "GO_TO_LOCATION":
            target = result.parameters.get("target", "")
            # Always reject completely empty targets (no target at all)
            if not target:
                return "Which location should I go to? Please specify a target like 'shelf_a', 'pallet_1', or 'landing_pad'."
            # Always reject UNKNOWN targets - LLM couldn't find matching object
            if target.upper() == "UNKNOWN":
                return "I couldn't find that location in the environment. Please specify a valid target like 'shelf_a', 'pallet_1', or 'forklift'."
        
        # Reject low confidence intents
        if result.confidence < self.confidence_threshold:
            return f"I'm not confident I understood correctly. Did you mean '{result.intent_type}'? Please confirm or rephrase."
        
        return ""  # No rejection
    
    def _publish_user_feedback(self, original_cmd: str, feedback_type: str, 
                               message: str, confidence: float):
        """Publish feedback to user (for rejections, clarifications, etc.)"""
        msg = UserFeedback()
        msg.original_command = original_cmd
        msg.feedback_type = feedback_type
        msg.message = message
        msg.confidence = confidence
        msg.stamp = self.get_clock().now().to_msg()
        
        self.user_feedback_pub.publish(msg)
        self.get_logger().info(f'Published user feedback: [{feedback_type}] {message}')

    def _extract_param(self, parameters: dict, key: str) -> str:
        """Extract a parameter value from the LLM result parameters dict"""
        return parameters.get(key, "")

    def _get_context_for_intent(self, intent_result) -> str:
        """
        Get context for intent - THESIS A/B COMPARISON.
        
        Both modes go through LGA:
        - use_lga=True:  Query LGA with baseline_mode=False (filtered context)
        - use_lga=False: Query LGA with baseline_mode=True  (all context)
        """
        import time
        
        # Track metrics
        self.metrics['total_requests'] += 1
        if self.use_lga:
            self.metrics['lga_requests'] += 1
        else:
            self.metrics['baseline_requests'] += 1
        
        # Build LGA request
        request = QueryLGA.Request()
        request.intent_type = intent_result.intent_type
        request.current_position = self.current_position
        
        # THESIS A/B: Set baseline_mode based on use_lga flag
        request.baseline_mode = not self.use_lga  # use_lga=True means baseline_mode=False
        
        # Set intent-specific fields
        if intent_result.intent_type == "GO_TO_LOCATION":
            target = self._extract_param(intent_result.parameters, "target")
            if not target:
                return None
            
            # Handle coordinate targets (from "go back" after MOVE_DIRECTION commands)
            # LLM may output coordinates like "(x, y, z)" when there's no named previous location
            if target.startswith('(') and target.endswith(')'):
                try:
                    coords = target.strip('()').split(',')
                    x, y, z = float(coords[0].strip()), float(coords[1].strip()), float(coords[2].strip())
                    self.get_logger().info(f"Target is coordinates: ({x}, {y}, {z}) - direct navigation")
                    # Return context for direct coordinate navigation (bypass LGA lookup)
                    return f"drone_pos: ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}); target_coords: ({x}, {y}, {z}); is_flying: True"
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f"Failed to parse coordinates '{target}': {e}")
            
            request.target_name = target
        elif intent_result.intent_type == "MOVE_DIRECTION":
            request.direction = self._extract_param(intent_result.parameters, "direction")
            distance_str = self._extract_param(intent_result.parameters, "distance")
            request.distance = float(distance_str) if distance_str else 1.0
        elif intent_result.intent_type == "COMPLEX_TASK":
            # For complex tasks, we want specific target context if available, 
            # otherwise we'll rely on the planner's raw command visibility.
            # But we should still try to get target context if a target is named.
            target = self._extract_param(intent_result.parameters, "target")
            if target:
                 request.target_name = target
        
        if not self.lga_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("LGA service not available")
            return None
        
        future = self.lga_client.call_async(request)
        
        try:
            start = time.time()
            while not future.done() and (time.time() - start) < 5.0:
                time.sleep(0.05)
            
            if not future.done():
                self.get_logger().error("LGA service call timed out")
                return None
        except Exception as e:
            self.get_logger().error(f"Error waiting for LGA: {e}")
            return None
        
        response = future.result()
        if response is None:
            self.get_logger().error("LGA service returned None")
            return None
        
        # Get context from LGA response
        context = response.filtered_context
        token_count = response.context_token_count
        self.metrics['total_context_tokens'] += token_count
        
        # Update current position from LGA (it has access to actual drone pose)
        self.current_position = response.drone_position
        
        # Log for thesis comparison
        mode = "BASELINE" if request.baseline_mode else "FILTERED"
        self.get_logger().info(f"[{mode}] {token_count} tokens from LGA")
        
        # Handle target not found case
        if intent_result.intent_type == "GO_TO_LOCATION" and not response.target_found:
            return f"Target '{request.target_name}' not found"
        
        return context
    
    def _publish_thesis_metrics(self, command, intent_results):
        """
        Publish thesis metrics for test runner to collect.
        
        Args:
            command: Original voice command
            intent_results: List of extracted IntentResult objects
        """
        # Build metrics message
        metrics_msg = ThesisMetrics()
        metrics_msg.test_command = command
        metrics_msg.source = "intent_extractor"
        metrics_msg.mode = "filtered" if self.use_lga else "baseline"
        
        # Intent info - comma-separated for multi-intent
        intent_types = [r.intent_type for r in intent_results]
        metrics_msg.intent_type = ",".join(intent_types)
        
        # Get target from first GO_TO_LOCATION intent
        target = ""
        for r in intent_results:
            if r.intent_type == "GO_TO_LOCATION":
                target = r.parameters.get('target', '')
                break
        metrics_msg.target = target
        
        # Get metrics from LLM interface
        metrics_msg.context_tokens = self.llm.last_metrics.get('input_tokens', 0)
        metrics_msg.intent_time_ms = self.llm.last_metrics.get('inference_time_ms', 0)
        
        # Get raw response from first intent result if available
        if intent_results and hasattr(intent_results[0], 'raw_response'):
            metrics_msg.llm_response = intent_results[0].raw_response
        
        # Planner metrics are 0 (filled by planner node)
        metrics_msg.planner_time_ms = 0
        metrics_msg.action_count = 0
        
        # Timestamp
        metrics_msg.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self.metrics_pub.publish(metrics_msg)
        self.get_logger().info(
            f"[THESIS] intent_time={metrics_msg.intent_time_ms}ms, "
            f"tokens={metrics_msg.context_tokens}, target={target}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IntentExtractorNode()
    
    # Use MultiThreadedExecutor to allow service calls from callbacks
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
