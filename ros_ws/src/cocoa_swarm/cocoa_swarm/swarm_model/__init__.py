"""
SwarmModel — tracks live position/state of every drone in the swarm.

Usage (inside a ROS2 Node):
    model = SwarmModel(node, drone_ids=["cf231", "cf232"])
    pos = model.get_state("cf231")  # → {"x": 0.1, "y": -0.2, "z": 1.0}
"""

from geometry_msgs.msg import PoseStamped


class DroneState:
    """Container for a single drone's live state."""

    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.connected: bool = False

    def update_pose(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.connected = True

    def to_dict(self):
        return {
            "drone_id": self.drone_id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "connected": self.connected,
        }


class SwarmModel:
    """
    Subscribes to /{drone_id}/pose for each drone and maintains live state.

    Instantiate once and pass a ROS2 node to register pose subscriptions.
    """

    def __init__(self, node, drone_ids: list[str]):
        self._node = node
        self._states: dict[str, DroneState] = {}

        for did in drone_ids:
            state = DroneState(did)
            self._states[did] = state
            node.create_subscription(
                PoseStamped,
                f"/{did}/pose",
                lambda msg, d=did: self._states[d].update_pose(msg),
                10,
            )
            node.get_logger().info(f"[SwarmModel] Tracking /{did}/pose")

    def get_state(self, drone_id: str) -> dict:
        """Return current state dict for a drone. Returns empty dict if unknown."""
        if drone_id in self._states:
            return self._states[drone_id].to_dict()
        return {}

    def get_position(self, drone_id: str):
        """Return (x, y, z) tuple for a drone, or None if unknown."""
        if drone_id in self._states and self._states[drone_id].connected:
            s = self._states[drone_id]
            return (s.x, s.y, s.z)
        return None

    def get_all_states(self) -> dict[str, dict]:
        """Return state dict for every tracked drone."""
        return {did: s.to_dict() for did, s in self._states.items()}

    def all_connected(self) -> bool:
        """True if all tracked drones have received at least one pose message."""
        return all(s.connected for s in self._states.values())
