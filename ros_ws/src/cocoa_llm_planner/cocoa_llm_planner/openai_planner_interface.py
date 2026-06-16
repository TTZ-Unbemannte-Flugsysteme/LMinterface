"""
OpenAI Interface for LLM Planner
Uses OpenAI's chat completions API for action plan generation
"""

import json
import os
import re
import time
from typing import List, Dict, Any, Optional

# Import openai - will need to be installed: pip install openai
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


class PlannerOpenAIInterface:
    """
    OpenAI interface for the LLM Planner.
    Generates action plans from intents using GPT-4o or similar models.
    """
    
    def __init__(
        self,
        logger,
        api_key: str = None,
        model: str = "gpt-4o-mini",
        temperature: float = 0.0,
        max_tokens: int = 1024,
    ):
        """
        Initialize OpenAI interface for planner
        
        Args:
            logger: ROS logger for logging
            api_key: OpenAI API key (or set OPENAI_API_KEY env var)
            model: Model to use
            temperature: Sampling temperature (0.0 for deterministic)
            max_tokens: Max tokens to generate
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("openai package not installed. Run: pip install openai")
        
        self.logger = logger
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        
        # Get API key from parameter or environment
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not provided. Set OPENAI_API_KEY env var or pass api_key parameter.")
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.api_key)
        
        # Metrics storage
        self.last_metrics = {
            'inference_time_ms': 0,
            'input_tokens': 0,
            'plan_valid': False,
            'action_count': 0
        }
        
        self.logger.info(f"Planner OpenAI interface initialized with model: {model}")

    def generate_plan(
        self,
        system_prompt: str,
        user_prompt: str,
        use_cot: bool = False
    ) -> str:
        """
        Generate action plan using OpenAI API.
        
        Args:
            system_prompt: The planner system prompt with tools definition
            user_prompt: The formatted intent and context
            use_cot: Whether to use chain-of-thought prompting
            
        Returns:
            Raw LLM response text
        """
        # Build messages
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]
        
        # Estimate input tokens
        input_tokens = (len(system_prompt) + len(user_prompt)) // 4
        
        try:
            start_time = time.time()
            
            # Call OpenAI API with JSON mode for reliable parsing
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
            
            self.logger.info(f"OpenAI planner response: {raw_text[:200]}...")
            self.logger.info(f"Planner took {inference_time_ms}ms, {actual_tokens} tokens")
            
            # Store metrics
            self.last_metrics = {
                'inference_time_ms': inference_time_ms,
                'input_tokens': actual_tokens,
                'plan_valid': True,
                'action_count': 0  # Will be updated after parsing
            }
            
            return raw_text
            
        except Exception as e:
            self.logger.error(f"OpenAI API call failed: {e}")
            self.last_metrics = {
                'inference_time_ms': 0,
                'input_tokens': input_tokens,
                'plan_valid': False,
                'action_count': 0
            }
            return ""

    def update_action_count(self, count: int):
        """Update the action count in metrics after parsing"""
        self.last_metrics['action_count'] = count
