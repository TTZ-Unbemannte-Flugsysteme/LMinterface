"""
LLM Interface for Cocoa Intent - OpenAI API version
Uses OpenAI's chat completions API (GPT-4, GPT-4o, etc.)
"""

import json
import os
import re
import time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from rclpy.node import Node

# Import openai - will need to be installed: pip install openai
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


@dataclass
class IntentResult:
    """Result from intent extraction"""
    intent_type: str
    parameters: Dict[str, Any]
    confidence: float
    raw_response: str


class OpenAIInterface:
    """
    LLM Interface using OpenAI's API for intent extraction.
    Uses GPT-4o-mini by default (fast and cheap), can be changed to GPT-4o for better accuracy.
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
        api_key: str = None,
        model: str = "gpt-4o-mini",  # Options: gpt-4o-mini, gpt-4o, gpt-4-turbo
        temperature: float = 0.0,
        max_tokens: int = 256,
    ):
        """
        Initialize OpenAI interface
        
        Args:
            node: ROS2 node for logging
            system_prompt: System prompt for intent extraction
            api_key: OpenAI API key (or set OPENAI_API_KEY env var)
            model: Model to use (gpt-4o-mini is recommended for speed/cost)
            temperature: Sampling temperature (0.0 for deterministic)
            max_tokens: Max tokens to generate
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("openai package not installed. Run: pip install openai")
        
        self.node = node
        self.system_prompt = system_prompt
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.logger = node.get_logger()
        
        # Get API key from parameter or environment
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not provided. Set OPENAI_API_KEY env var or pass api_key parameter.")
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.api_key)
        
        # Metrics for thesis logging
        self.last_metrics = {
            'inference_time_ms': 0,
            'input_tokens': 0
        }
        
        self.logger.info(f"OpenAI interface initialized with model: {model}")

    def extract_intent(self, user_text: str, conversation_history: str = "",
                        environment_knowledge: str = "", 
                        assume_ambiguous: bool = True,
                        drone_position: tuple = None,
                        use_cot: bool = False) -> List[IntentResult]:
        """
        Extract intent(s) from user text using OpenAI API
        
        Args:
            user_text: The user's spoken command
            conversation_history: Optional formatted history of recent exchanges
            environment_knowledge: Optional serialized EKG knowledge for object disambiguation
            assume_ambiguous: If True, LLM picks best match for ambiguous refs
            drone_position: Optional (x, y, z) tuple of drone's current position
            use_cot: If True, enables Chain-of-Thought reasoning (less needed with GPT-4)
            
        Returns:
            List of IntentResult (may contain multiple for compound commands)
        """
        # Build system message
        system_content = self.system_prompt
        
        # Add environment knowledge to system prompt
        if environment_knowledge:
            system_content += f"\n\n{environment_knowledge}\n"
            if assume_ambiguous:
                system_content += "\nDISAMBIGUATION MODE: ASSUME - Pick the most likely object when reference is ambiguous.\n"
            else:
                system_content += "\nDISAMBIGUATION MODE: STRICT - Output UNKNOWN if target is ambiguous.\n"
        
        # Add conversation history
        if conversation_history:
            system_content += f"\n{conversation_history}\n"
        
        # Add drone position
        if drone_position:
            system_content += f"\nDRONE CURRENT POSITION: ({drone_position[0]:.1f}, {drone_position[1]:.1f}, {drone_position[2]:.1f})\n"
        
        # Build messages for chat completion
        messages = [
            {"role": "system", "content": system_content},
            {"role": "user", "content": user_text}
        ]
        
        # Estimate input tokens (rough)
        input_tokens = len(system_content + user_text) // 4
        
        try:
            start_time = time.time()
            
            # Call OpenAI API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_completion_tokens=self.max_tokens,  # Use max_completion_tokens for newer models
                response_format={"type": "json_object"}  # Force JSON output
            )
            
            inference_time_ms = int((time.time() - start_time) * 1000)
            
            raw_text = response.choices[0].message.content.strip()
            actual_tokens = response.usage.total_tokens if response.usage else input_tokens
            
            self.logger.info(f"LLM raw response: {raw_text}")
            self.logger.info(f"Intent extraction took {inference_time_ms}ms, {actual_tokens} tokens")
            
            # Parse JSON response
            intents = self._parse_response(raw_text)
            
            # Store metrics
            self.last_metrics = {
                'inference_time_ms': inference_time_ms,
                'input_tokens': actual_tokens
            }
            
            return intents
            
        except Exception as e:
            self.logger.error(f"OpenAI API call failed: {e}")
            self.last_metrics = {'inference_time_ms': 0, 'input_tokens': input_tokens}
            return []

    def reason_about_safety(self, intent_type: str, parameters: Dict[str, Any], 
                            context: str) -> tuple:
        """
        Ask LLM to reason about whether an action is safe to execute.
        
        Returns:
            Tuple of (should_execute: bool, reason: str)
        """
        safety_prompt = f"""You are a drone safety system. Analyze if this action is safe.

ACTION: {intent_type}
PARAMETERS: {parameters}
CONTEXT: {context}

Rules:
- Battery below 10% = NOT SAFE
- Critical warning present = NOT SAFE  
- Obstacles with no path = NOT SAFE
- Otherwise = SAFE

Respond with JSON: {{"safe": true/false, "reason": "brief explanation"}}"""

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a drone safety analyzer. Output only JSON."},
                    {"role": "user", "content": safety_prompt}
                ],
                temperature=0.0,
                max_completion_tokens=100,
                response_format={"type": "json_object"}
            )
            
            raw_text = response.choices[0].message.content.strip()
            self.logger.info(f"Safety reasoning response: {raw_text}")
            
            data = json.loads(raw_text)
            return (data.get("safe", True), data.get("reason", ""))
            
        except Exception as e:
            self.logger.error(f"Safety reasoning failed: {e}")
            return (True, "")  # Default to safe on error

    def _parse_response(self, raw_text: str) -> List[IntentResult]:
        """Parse LLM response into list of IntentResults"""
        try:
            text = raw_text.strip()
            
            # Handle markdown code blocks if present
            json_block_match = re.search(r'```json\s*(.*?)\s*```', text, re.DOTALL)
            if json_block_match:
                text = json_block_match.group(1).strip()
            
            # Parse JSON
            data = json.loads(text)
            
            if not isinstance(data, dict):
                self.logger.error(f"Expected JSON object, got: {type(data).__name__}")
                return [IntentResult("UNKNOWN", {}, 0.0, raw_text)]
            
            # Handle array format with "intents" key
            if "intents" in data:
                intent_list = data["intents"]
                if not isinstance(intent_list, list):
                    intent_list = [intent_list]
            else:
                intent_list = [data]
            
            results = []
            for intent_data in intent_list:
                intent_type = intent_data.get("intent_type", "UNKNOWN")
                parameters = intent_data.get("parameters", {})
                confidence = intent_data.get("confidence", 0.95)
                
                if intent_type not in self.VALID_INTENTS:
                    self.logger.warn(f"Invalid intent type: {intent_type}, defaulting to UNKNOWN")
                    intent_type = "UNKNOWN"
                
                results.append(IntentResult(
                    intent_type=intent_type,
                    parameters=parameters,
                    confidence=confidence,
                    raw_response=raw_text
                ))
            
            return results if results else [IntentResult("UNKNOWN", {}, 0.0, raw_text)]
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON: {e}")
            return [IntentResult("UNKNOWN", {}, 0.0, raw_text)]
