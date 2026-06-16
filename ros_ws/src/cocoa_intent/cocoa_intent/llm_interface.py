"""
LLM Interface for Cocoa Intent
Wrapper around llama-cpp-python for intent extraction
"""

import json
import os
from typing import Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class IntentResult:
    """Result from intent extraction"""
    intent_type: str
    parameters: Dict[str, Any]
    confidence: float
    raw_response: str


class LLMInterface:
    """
    LLM Interface for intent classification
    Uses llama-cpp-python for local inference
    """
    
    VALID_INTENTS = {
        "MOVE_DIRECTION", "GO_TO_LOCATION", "CHANGE_ALTITUDE",
        "ROTATE", "TAKEOFF", "LAND", "HOVER", 
        "EMERGENCY_STOP", "QUERY", "UNKNOWN"
    }
    
    def __init__(
        self,
        model_path: str,
        system_prompt: str,
        n_gpu_layers: int = 0,
        n_ctx: int = 4096,
        temperature: float = 0.0,
        max_tokens: int = 256,
        logger = None
    ):
        """
        Initialize LLM interface
        
        Args:
            model_path: Path to GGUF model file
            system_prompt: System prompt for intent extraction
            n_gpu_layers: GPU layers (0 for CPU, -1 for all)
            n_ctx: Context window size
            temperature: Sampling temperature
            max_tokens: Max tokens to generate
            logger: ROS2 logger
        """
        self.system_prompt = system_prompt
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.logger = logger
        
        self._log_info(f"Loading LLM from: {model_path}")
        self._log_info(f"GPU layers: {n_gpu_layers}, Context: {n_ctx}")
        
        try:
            from llama_cpp import Llama
            self.llm = Llama(
                model_path=model_path,
                n_gpu_layers=n_gpu_layers,
                n_ctx=n_ctx,
                n_threads=os.cpu_count(),
                verbose=False,
            )
            self._log_info("LLM loaded successfully")
        except Exception as e:
            self._log_error(f"Failed to load LLM: {e}")
            raise

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def extract_intent(self, user_text: str) -> Optional[IntentResult]:
        """
        Extract intent from user text
        
        Args:
            user_text: The user's spoken command
            
        Returns:
            IntentResult or None on failure
        """
        # Build the full prompt
        full_prompt = f"{self.system_prompt}\nUser: \"{user_text}\"\n"
        
        try:
            response = self.llm(
                full_prompt,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                stop=["User:", "\n\n"],
            )
            
            raw_text = response["choices"][0]["text"].strip()
            self._log_info(f"LLM raw response: {raw_text}")
            
            # Parse JSON response
            result = self._parse_response(raw_text)
            return result
            
        except Exception as e:
            self._log_error(f"LLM inference failed: {e}")
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
            # Find the first { and match the closing }
            json_start = text.find('{')
            if json_start != -1:
                # Count braces to find the matching closing brace
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
                    if json_end < len(raw_text.strip()):
                        self._log_warn(f"Trimmed extra text after JSON: '{raw_text[json_end:json_end+50]}...'")
            
            # Parse JSON
            data = json.loads(text)
            
            intent_type = data.get("intent_type", "UNKNOWN")
            parameters = data.get("parameters", {})
            confidence = data.get("confidence", 0.5)
            
            # Validate intent type
            if intent_type not in self.VALID_INTENTS:
                self._log_warn(f"Invalid intent type: {intent_type}, defaulting to UNKNOWN")
                intent_type = "UNKNOWN"
            
            return IntentResult(
                intent_type=intent_type,
                parameters=parameters,
                confidence=confidence,
                raw_response=raw_text
            )
            
        except json.JSONDecodeError as e:
            self._log_error(f"Failed to parse JSON: {e}")
            self._log_error(f"Raw text was: {raw_text}")
            # Return UNKNOWN on parse failure
            return IntentResult(
                intent_type="UNKNOWN",
                parameters={},
                confidence=0.0,
                raw_response=raw_text
            )
