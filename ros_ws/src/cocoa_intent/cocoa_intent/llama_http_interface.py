"""
LLM Interface for Cocoa Intent - HTTP Server version
Uses llama.cpp HTTP server for shared LLM inference
"""

import json
import re
import requests
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from rclpy.node import Node


@dataclass
class IntentResult:
    """Result from intent extraction"""
    intent_type: str
    parameters: Dict[str, Any]
    confidence: float
    raw_response: str


class LlamaHTTPInterface:
    """
    LLM Interface using llama.cpp HTTP server for shared inference.
    Connects to http://localhost:8081/v1/completions
    """
    
    VALID_INTENTS = {
        "MOVE_DIRECTION", "GO_TO_LOCATION", "CHANGE_ALTITUDE",
        "ROTATE", "TAKEOFF", "LAND", "HOVER", 
        "EMERGENCY_STOP", "QUERY", "COMPLEX_TASK", "UNKNOWN"
    }
    
    def __init__(
        self,
        node: Node,
        system_prompt: str,
        server_url: str = "http://localhost:8081",
        temperature: float = 0.0,
        max_tokens: int = 256,
    ):
        """
        Initialize llama.cpp HTTP interface
        
        Args:
            node: ROS2 node for logging
            system_prompt: System prompt for intent extraction
            server_url: URL of llama.cpp server
            temperature: Sampling temperature
            max_tokens: Max tokens to generate
        """
        self.node = node
        self.system_prompt = system_prompt
        self.server_url = server_url
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.logger = node.get_logger()
        
        # Metrics for thesis logging
        self.last_metrics = {
            'inference_time_ms': 0,
            'input_tokens': 0
        }
        
        # Test connection
        try:
            resp = requests.get(f"{server_url}/health", timeout=5)
            if resp.status_code == 200:
                self.logger.info(f"Connected to llama.cpp server at {server_url}")
            else:
                self.logger.warn(f"Server returned status {resp.status_code}")
        except Exception as e:
            self.logger.warn(f"Could not connect to llama.cpp server: {e}")
            self.logger.info("Start server with: python -m llama_cpp.server --model <path> --n_gpu_layers 32")

    def extract_intent(self, user_text: str, conversation_history: str = "",
                        environment_knowledge: str = "", 
                        assume_ambiguous: bool = True,
                        drone_position: tuple = None,
                        use_cot: bool = False) -> List[IntentResult]:
        """
        Extract intent(s) from user text using llama.cpp HTTP server (Chat API)
        
        Args:
            user_text: The user's spoken command
            conversation_history: Optional formatted history of recent exchanges
            environment_knowledge: Optional serialized EKG knowledge for object disambiguation
            assume_ambiguous: If True, LLM picks best match for ambiguous refs; if False, outputs UNKNOWN
            drone_position: Optional (x, y, z) tuple of drone's current position
            use_cot: If True, enables Chain-of-Thought reasoning
            
        Returns:
            List of IntentResult (may contain multiple for compound commands)
        """
        # Prepare System Prompt (Dynamic handling for CoT)
        base_system_prompt = self.system_prompt
        
        if use_cot:
            self.logger.info("CoT Mode Enabled: Stripping strict JSON examples from prompt.")
            # 1. Strip Strict JSON Examples and Rules to remove bias
            if "EXAMPLES:" in base_system_prompt:
                base_system_prompt = base_system_prompt.split("EXAMPLES:")[0]
            
            # 2. Inject CoT Examples (emphasizing Relationships and Multi-Intent)
            cot_examples = """
EXAMPLES (Chain-of-Thought):

User: "go to shelf A"
<thought>
1. User intent: Go to location.
2. Target: "shelf A". Matches object 'shelf_a'.
3. Action: GO_TO_LOCATION shelf_a.
</thought>
```json
{"intents": [{"intent_type": "GO_TO_LOCATION", "parameters": {"target": "shelf_a"}, "confidence": 0.95}]}
```

User: "go up 2 meters"
<thought>
1. User intent: Change altitude upward.
2. Direction: "up". Distance: 2.0 meters.
3. Extract as CHANGE_ALTITUDE with direction=up, distance=2.0.
</thought>
```json
{"intents": [{"intent_type": "CHANGE_ALTITUDE", "parameters": {"direction": "up", "distance": 2.0, "unit": "meters"}, "confidence": 0.95}]}
```

User: "move forward 3 meters"
<thought>
1. User intent: Move in a direction relative to current position.
2. Direction: "forward". Distance: 3.0 meters.
3. Extract as MOVE_DIRECTION with direction=forward, distance=3.0.
</thought>
```json
{"intents": [{"intent_type": "MOVE_DIRECTION", "parameters": {"direction": "forward", "distance": 3.0, "unit": "meters"}, "confidence": 0.95}]}
```

User: "go to the pallet near shelf A"
<thought>
1. User intent: Go to location.
2. Target: "pallet" with relation "near shelf A".
3. Check Environment Relationships: Found 'pallet_1' is near 'shelf_a'.
4. Resolved Target: 'pallet_1'.
</thought>
```json
{"intents": [{"intent_type": "GO_TO_LOCATION", "parameters": {"target": "pallet_1", "relation": "near"}, "confidence": 0.95}]}
```

User: "take off and go near shelf B"
<thought>
1. Multi-intent detected.
2. First: "take off" -> TAKEOFF.
3. Second: "go near shelf B" -> GO_TO_LOCATION.
4. Target: "shelf B" (shelf_b). Relation: "near".
</thought>
```json
{"intents": [{"intent_type": "TAKEOFF", "parameters": {}, "confidence": 0.95}, {"intent_type": "GO_TO_LOCATION", "parameters": {"target": "shelf_b", "relation": "near"}, "confidence": 0.95}]}
```
"""
            base_system_prompt += cot_examples

        # Build system content
        system_content = base_system_prompt
        
        if environment_knowledge:
            system_content += f"\n\n{environment_knowledge}\n"
            if assume_ambiguous:
                system_content += "\nDISAMBIGUATION MODE: ASSUME - Pick the most likely object when reference is ambiguous.\n"
            else:
                system_content += "\nDISAMBIGUATION MODE: STRICT - Output UNKNOWN if target is ambiguous.\n"
        
        if conversation_history:
            system_content += f"\n{conversation_history}\n"
        
        # Add current drone position
        if drone_position:
            system_content += f"\nDRONE CURRENT POSITION: ({drone_position[0]:.1f}, {drone_position[1]:.1f}, {drone_position[2]:.1f})\n"
        
        # Inject CoT instructions
        if use_cot:
            system_content += "\nINSTRUCTIONS: First, THINK step-by-step in a <thought> block.\n"
            system_content += "1. Analyze the user command against the Environment Knowledge.\n"
            system_content += "2. Check for explicit RELATIONSHIPS (e.g. 'near shelf A') - these override proximity.\n"
            system_content += "3. Check PROXIMITY only if no specific relation is mentioned.\n"
            system_content += "4. If no distance is specified for MOVE_DIRECTION, set 'distance' to 1.0.\n"
            system_content += "5. Finally, output the JSON object.\n"
            system_content += "Format: <thought>Reasoning...</thought>\n```json\n{\"intent_type\": \"TYPE\", ...}\n```\n"
        else:
            system_content += "\nINSTRUCTIONS: Output ONLY the JSON object. Do not include any reasoning or thought process.\n"
        
        # Build messages for Chat API
        messages = [
            {"role": "system", "content": system_content},
            {"role": "user", "content": user_text}
        ]
        
        # Estimate input tokens
        input_tokens = len(system_content + user_text) // 4
        
        try:
            # Track inference time
            import time
            start_time = time.time()
            
            # Determine stop tokens
            stop_tokens = ["User:", "<|im_end|>"]
            if not use_cot:
                stop_tokens.insert(0, "\n\n")
            
            # Call HTTP Chat API
            # NOTE: Llama.cpp server supports OpenAI-compatible chat completions
            response = requests.post(
                f"{self.server_url}/v1/chat/completions",
                json={
                    "messages": messages,
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "stop": stop_tokens,
                    # "response_format": {"type": "json_object"} # Optional: strictly enforce JSON if model supports it
                },
                timeout=60
            )
            
            # Calculate inference time
            inference_time_ms = int((time.time() - start_time) * 1000)
            
            if response.status_code != 200:
                self.logger.error(f"HTTP error: {response.status_code}")
                self.logger.error(f"Response: {response.text}")
                self.last_metrics = {'inference_time_ms': inference_time_ms, 'input_tokens': input_tokens}
                return []
            
            result = response.json()
            raw_text = result["choices"][0]["message"]["content"].strip()
            
            self.logger.info(f"LLM raw response: {raw_text}")
            self.logger.info(f"Intent extraction took {inference_time_ms}ms, ~{input_tokens} tokens")
            
            # Parse JSON response
            intents = self._parse_response(raw_text)
            
            # Store metrics
            self.last_metrics = {
                'inference_time_ms': inference_time_ms,
                'input_tokens': input_tokens
            }
            
            return intents
            
        except requests.exceptions.Timeout:
            self.logger.error("LLM request timed out")
            self.last_metrics = {'inference_time_ms': 60000, 'input_tokens': input_tokens}
            return []
        except Exception as e:
            self.logger.error(f"LLM inference failed: {e}")
            self.last_metrics = {'inference_time_ms': 0, 'input_tokens': input_tokens}
            return []

    def reason_about_safety(self, intent_type: str, parameters: Dict[str, Any], 
                            context: str) -> tuple:
        """
        Ask LLM to reason about whether an action is safe to execute given context.
        
        Args:
            intent_type: The extracted intent type
            parameters: Intent parameters
            context: LGA context string (battery, obstacles, warnings, etc.)
            
        Returns:
            Tuple of (should_execute: bool, reason: str)
        """
        safety_system = "You are a drone safety system. Output only valid JSON."
        safety_user = f"""
ACTION: {intent_type}
PARAMETERS: {parameters}
CONTEXT: {context}

Analyze the context.
Rules:
- Battery < 10% = NOT SAFE
- Critical warning = NOT SAFE
- Obstacles with no path = NOT SAFE
- Otherwise = SAFE

Respond with JSON: {{"safe": true/false, "reason": "brief explanation"}}
"""

        try:
            response = requests.post(
                f"{self.server_url}/v1/chat/completions",
                json={
                    "messages": [
                        {"role": "system", "content": safety_system},
                        {"role": "user", "content": safety_user}
                    ],
                    "max_tokens": 100,
                    "temperature": 0.0,
                    "stop": ["\n\n"],
                    # "response_format": {"type": "json_object"}
                },
                timeout=30
            )
            
            if response.status_code != 200:
                self.logger.error(f"Safety reasoning HTTP error: {response.status_code}")
                return (True, "")  # Default to safe on error
            
            result = response.json()
            raw_text = result["choices"][0]["message"]["content"].strip()
            self.logger.info(f"Safety reasoning response: {raw_text}")
            
            # Parse JSON response
            try:
                data = json.loads(raw_text)
                is_safe = data.get("safe", True)
                reason = data.get("reason", "")
                return (is_safe, reason)
            except json.JSONDecodeError:
                self.logger.warn(f"Could not parse safety response: {raw_text}")
                return (True, "")  # Default to safe on parse error
                
        except Exception as e:
            self.logger.error(f"Safety reasoning failed: {e}")
            return (True, "")  # Default to safe on exception

    def _parse_response(self, raw_text: str) -> List[IntentResult]:
        """Parse LLM response into list of IntentResults"""
        try:
            # Clean up response - extract JSON
            text = raw_text.strip()
            
            # Robustly find markdown code blocks
            # Search for ```json ... ```
            import re
            json_block_match = re.search(r'```json\s*(.*?)\s*```', text, re.DOTALL)
            if json_block_match:
                text = json_block_match.group(1).strip()
            else:
                # Search for generic ``` ... ``` to be safe
                code_block_match = re.search(r'```\s*(.*?)\s*```', text, re.DOTALL)
                if code_block_match:
                     text = code_block_match.group(1).strip()
                else:
                    # Fallback: Find the LAST brace group if it looks like the main JSON
                    # This avoids grabbing { } inside <thought> blocks at the start
                    # Look for the last JSON object in the string
                    last_brace_start = text.rfind('{')
                    if last_brace_start != -1:
                        # Extract from there to end or finding matching brace
                        # But rfind might find the LAST item in the intent list if formatted oddly
                        # Safer approach: Try to find start of JSON by looking for {" or {\n
                        pass
                        
                    # Standard extraction: find first { that IS NOT inside <thought>
                    # Easier hack: Split by </thought> and take right side
                    if "</thought>" in text:
                        text = text.split("</thought>")[-1].strip()
                    
                    # Now extract JSON object
                    json_start = text.find('{')
                    if json_start != -1:
                        brace_count = 0
                        json_end = -1
                        for i, char in enumerate(text[json_start:], start=json_start):
                            if char == '{':
                                brace_count += 1
                            elif char == '}':
                                brace_count -= 1
                                if brace_count == 0:
                                    json_end = i + 1
                                    break
                        
                        if json_end != -1:
                            text = text[json_start:json_end]
            
            # Parse JSON
            data = json.loads(text)
            
            # Validate we got a dict (not a list or primitive)
            if not isinstance(data, dict):
                self.logger.error(f"Expected JSON object, got: {type(data).__name__}")
                return [IntentResult(
                    intent_type="UNKNOWN",
                    parameters={},
                    confidence=0.0,
                    raw_response=raw_text
                )]
            
            # Handle new array format with "intents" key
            if "intents" in data:
                intent_list = data["intents"]
                # Validate intent_list is a list of dicts
                if not isinstance(intent_list, list):
                    intent_list = [intent_list]
            else:
                # Backward compatibility: single intent
                intent_list = [data]
            
            results = []
            for intent_data in intent_list:
                intent_type = intent_data.get("intent_type", "UNKNOWN")
                parameters = intent_data.get("parameters", {})
                confidence = intent_data.get("confidence", 0.5)
                
                # Validate intent type
                if intent_type not in self.VALID_INTENTS:
                    self.logger.warn(f"Invalid intent type: {intent_type}, defaulting to UNKNOWN")
                    intent_type = "UNKNOWN"
                
                results.append(IntentResult(
                    intent_type=intent_type,
                    parameters=parameters,
                    confidence=confidence,
                    raw_response=raw_text
                ))
            
            return results if results else [IntentResult(
                intent_type="UNKNOWN",
                parameters={},
                confidence=0.0,
                raw_response=raw_text
            )]
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON: {e}")
            self.logger.error(f"Raw text was: {raw_text}")
            return [IntentResult(
                intent_type="UNKNOWN",
                parameters={},
                confidence=0.0,
                raw_response=raw_text
            )]
