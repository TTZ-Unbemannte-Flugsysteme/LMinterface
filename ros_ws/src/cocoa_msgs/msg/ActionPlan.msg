# ActionPlan message - LLM output defining what BT Manager should execute
# Published by: llm_node
# Subscribed by: bt_manager_node

string[] action_sequence      # Ordered actions: ["takeoff", "move_forward", "hover", "land"]
float64[] action_parameters   # Parameters for each action: [1.0, 2.0, 0.0, 0.0]
string[] parameter_units      # Units for parameters: ["meters", "meters", "", ""]
string reasoning              # LLM's reasoning chain (for debugging/logging)
bool requires_confirmation    # True if action needs user approval before execution
float64 estimated_duration    # Estimated execution time in seconds
