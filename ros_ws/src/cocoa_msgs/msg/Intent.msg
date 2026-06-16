# Intent message - carries parsed user intent(s) from intent_extractor to planner
# Published by: intent_extractor_node
# Subscribed by: llm_planner_node
#
# Supports compound commands like "go up 5m and go to table" with multiple intents

string raw_command                  # Original user speech
string conversation_history         # Formatted history of recent exchanges (for COMPLEX_TASK)
cocoa_msgs/SingleIntent[] intents   # Array of intents (1 or more)
builtin_interfaces/Time stamp       # Timestamp of extraction
