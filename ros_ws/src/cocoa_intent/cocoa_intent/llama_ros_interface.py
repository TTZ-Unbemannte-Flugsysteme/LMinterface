"""
LLM Interface for Cocoa Intent - llama_ros version
Uses llama_ros ROS2 action for shared LLM inference
"""

import json
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from llama_msgs.action import GenerateResponse


@dataclass
class IntentResult:
    """Result from intent extraction"""
    intent_type: str
    parameters: Dict[str, Any]
    confidence: float
    raw_response: str


class LlamaROSInterface:
    """
    LLM Interface using llama_ros for shared inference.
    Connects to the /llama/generate_response action server.
    """
    
    VALID_INTENTS = {
        "MOVE_DIRECTION", "GO_TO_LOCATION", "CHANGE_ALTITUDE",
        "ROTATE", "TAKEOFF", "LAND", "HOVER", 
        "EMERGENCY_STOP", "QUERY", "UNKNOWN"
    }
    
    def __init__(
        self,
        node: Node,
        system_prompt: str,
        action_name: str = "/llama/generate_response",
        temperature: float = 0.0,
        max_tokens: int = 256,
    ):
        """
        Initialize llama_ros interface
        
        Args:
            node: ROS2 node to attach the action client
            system_prompt: System prompt for intent extraction
            action_name: Name of the llama_ros action
            temperature: Sampling temperature
            max_tokens: Max tokens to generate
        """
        self.node = node
        self.system_prompt = system_prompt
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.logger = node.get_logger()
        
        # Create action client
        self._action_client = ActionClient(
            node, GenerateResponse, action_name
        )
        
        self.logger.info(f"Waiting for llama_ros action server: {action_name}")
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.logger.error("llama_ros action server not available!")
            raise RuntimeError("llama_ros action server not available")
        self.logger.info("Connected to llama_ros action server")

    def extract_intent(self, user_text: str) -> Optional[IntentResult]:
        """
        Extract intent from user text using llama_ros
        
        Args:
            user_text: The user's spoken command
            
        Returns:
            IntentResult or None on failure
        """
        # Build the full prompt
        full_prompt = f"{self.system_prompt}\nUser: \"{user_text}\"\n"
        
        try:
            # Create action goal
            goal = GenerateResponse.Goal()
            goal.prompt = full_prompt
            goal.reset = True  # Clear KV cache for each new request
            goal.sampling_config.temp = self.temperature
            goal.stop = ["\n\n", "User:", "<|im_end|>"]  # Stop after JSON response
            
            # Send goal and wait for result
            self.logger.info("Sending prompt to llama_ros...")
            send_goal_future = self._action_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            start = time.time()
            while not send_goal_future.done() and (time.time() - start) < 30.0:
                time.sleep(0.05)
            
            if not send_goal_future.done():
                self.logger.error("Timeout waiting for goal acceptance")
                return None
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.logger.error("Goal was rejected by llama_ros")
                return None
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            
            start = time.time()
            while not get_result_future.done() and (time.time() - start) < 120.0:
                time.sleep(0.1)
            
            if not get_result_future.done():
                self.logger.error("Timeout waiting for LLM response")
                return None
            
            result = get_result_future.result()
            raw_text = result.result.response.text.strip()
            
            self.logger.info(f"LLM raw response: {raw_text}")
            
            # Parse JSON response
            return self._parse_response(raw_text)
            
        except Exception as e:
            self.logger.error(f"LLM inference failed: {e}")
            return None

    def _parse_response(self, raw_text: str) -> Optional[IntentResult]:
        """Parse LLM response into IntentResult"""
        try:
            # Clean up response - extract JSON
            text = raw_text.strip()
            
            # Handle markdown code blocks
            if text.startswith("```json"):
                text = text[7:]
            if text.startswith("```"):
                text = text[3:]
            if text.endswith("```"):
                text = text[:-3]
            text = text.strip()
            
            # Extract JSON object even if there's extra text after it
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
            
            intent_type = data.get("intent_type", "UNKNOWN")
            parameters = data.get("parameters", {})
            confidence = data.get("confidence", 0.5)
            
            # Validate intent type
            if intent_type not in self.VALID_INTENTS:
                self.logger.warn(f"Invalid intent type: {intent_type}, defaulting to UNKNOWN")
                intent_type = "UNKNOWN"
            
            return IntentResult(
                intent_type=intent_type,
                parameters=parameters,
                confidence=confidence,
                raw_response=raw_text
            )
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON: {e}")
            self.logger.error(f"Raw text was: {raw_text}")
            return IntentResult(
                intent_type="UNKNOWN",
                parameters={},
                confidence=0.0,
                raw_response=raw_text
            )
