# ThesisMetrics - Metrics for thesis experiments
# Published by: intent_extractor_node, llm_planner_node
# Subscribed by: test runner

string test_command        # Original voice command
string source              # "intent_extractor" or "planner"
string mode                # "filtered" or "baseline"

# Intent Extractor metrics
string intent_type         # Extracted intent (comma-separated for multi)
string target              # Extracted target object name
int32 context_tokens       # Number of tokens in context
int32 intent_time_ms       # Intent extraction inference time (ms)

# Planner metrics  
int32 planner_tokens       # Number of tokens in planner context
int32 planner_time_ms      # Planning inference time (ms)
int32 action_count         # Number of actions in plan
string llm_response        # Raw LLM response text for debugging


builtin_interfaces/Time stamp
