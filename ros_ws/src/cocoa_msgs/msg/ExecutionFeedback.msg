# ExecutionFeedback - reports action execution outcomes
# Published by: llm_planner_node
# Subscribed by: intent_extractor_node

string command           # Original user command
string intent_type       # The intent that was executed
bool success             # Overall success/failure
string[] actions         # List of actions attempted, e.g. ["takeoff", "goto"]
string[] results         # Result for each action, e.g. ["SUCCESS", "FAILED: timeout"]
string summary           # Human-readable summary, e.g. "2/3 actions succeeded"
builtin_interfaces/Time stamp
