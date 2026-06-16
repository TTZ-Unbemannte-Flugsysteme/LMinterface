"""
Tool definitions for the LLM Planner
Defines available drone actions with preconditions
"""

DRONE_TOOLS = {
    "takeoff": {
        "description": "Lift the drone to a specified height. Handles arming automatically for brushless drones. Use this first before any movement.",
        "preconditions": ["NOT is_flying"],
        "params": {
            "height": "float, meters above ground (default: 1.0, range: 0.3-2.0)"
        }
    },
    "land": {
        "description": "Land the drone safely at current position.",
        "preconditions": ["is_flying"],
        "params": {}
    },
    "goto": {
        "description": "Navigate DIRECTLY to specific coordinates (x, y, z). ERROR-PRONE if obstacles exist. USE THIS FOR: Relative moves (up, down, left, right, forward, back) and short adjustments. Ignores safety standoff.",
        "preconditions": ["is_flying"],
        "params": {
            "x": "float, x coordinate in meters",
            "y": "float, y coordinate in meters",
            "z": "float, z coordinate in meters (altitude)",
            "yaw": "float, heading in degrees (optional, default: 0)"
        }
    },
    "navigate_to": {
        "description": "Navigate SAFELY to coordinates using path planning. Uses A* to avoid obstacles. USE THIS FOR: Going to named locations (e.g., 'go to shelf'). WARNING: Keeps 2.0m standoff from target - DO NOT use for relative moves!",
        "preconditions": ["is_flying"],
        "params": {
            "x": "float, target x coordinate (from context)",
            "y": "float, target y coordinate (from context)",
            "z": "float, target z coordinate (flight height, typically 1.0)",
            "standoff": "float, optimal distance to keep from target (default: 2.0). Set to 0.0 for exact waypoints."
        }
    },
    "hover": {
        "description": "Hold current position for a duration. Use for stabilization or waiting.",
        "preconditions": ["is_flying"],
        "params": {
            "duration": "float, seconds to hover (default: 2.0)"
        }
    }
}


def get_tools_prompt() -> str:
    """Generate a formatted string of tool definitions for the LLM prompt"""
    lines = ["AVAILABLE TOOLS:"]
    for name, tool in DRONE_TOOLS.items():
        lines.append(f"\n- {name}: {tool['description']}")
        lines.append(f"  Preconditions: {', '.join(tool['preconditions'])}")
        if tool['params']:
            params_str = ", ".join([f"{k}: {v}" for k, v in tool['params'].items()])
            lines.append(f"  Parameters: {params_str}")
        else:
            lines.append("  Parameters: none")
    
    # Add important note about navigation
    lines.append("\n\nIMPORTANT TOOL SELECTION RULES:")
    lines.append("  1. User says 'go up/down/left/right/forward/back/move':")
    lines.append("     -> MUST use 'goto' (navigate_to will fail due to safety standoff!)")
    lines.append("  2. User says 'go to [object_name]':")
    lines.append("     -> MUST use 'navigate_to' (needs obstacle avoidance & safety standoff)")
    lines.append("  3. Never use 'navigate_to' for small adjustments (< 2m).")
    
    return "\n".join(lines)

