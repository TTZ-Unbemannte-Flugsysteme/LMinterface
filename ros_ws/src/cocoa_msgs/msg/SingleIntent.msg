# SingleIntent - individual intent within a compound command
# Used as an element in Intent.msg intents array

string intent_type          # MOVE_DIRECTION, GO_TO_LOCATION, TAKEOFF, etc.
string[] parameters         # Key-value pairs like "distance:2", "target:table"
float64 confidence          # Parser confidence 0.0-1.0
string context              # LGA context for this specific intent
