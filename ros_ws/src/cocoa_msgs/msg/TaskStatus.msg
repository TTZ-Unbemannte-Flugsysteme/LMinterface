# TaskStatus message - execution feedback from drone back to dialogue
# Published by: bt_manager_node
# Subscribed by: dialogue_manager for user feedback

string task_id                # Unique identifier for this task
string status                 # PENDING, EXECUTING, COMPLETED, FAILED, CANCELLED
float64 progress_percent      # 0.0 to 100.0
string current_action         # Current action being executed, e.g. "move_forward"
string message                # Human-readable status for user, e.g. "Moving... 50%"
builtin_interfaces/Time stamp # Timestamp of status update
