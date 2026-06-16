# Context message - filtered context prepared by LGA for LLM
# Published by: lga_node
# Used by: llm_node for action planning

string intent_summary         # Summarized intent from user
string[] relevant_objects     # Nearby objects from EKG, e.g. ["table", "wall", "door"]
float64[] object_distances    # Distances to each object in meters
string spatial_description    # Human-readable context, e.g. "Front: 3.2m clear, Right: wall at 0.5m"
string safety_constraints     # Any active safety constraints, e.g. "low_battery", "indoor_mode"
geometry_msgs/Point position  # Current drone position
