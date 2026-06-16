# UserFeedback - feedback to user when intent cannot be executed
# Published by: intent_extractor_node
# Subscribed by: user interface node (TTS, display, etc.)

string original_command    # What the user said
string feedback_type       # "REJECTION", "CLARIFICATION", "INFO"
string message             # LLM-generated explanation for user
float64 confidence
builtin_interfaces/Time stamp
