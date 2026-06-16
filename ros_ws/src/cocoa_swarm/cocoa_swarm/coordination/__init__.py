"""
ConflictResolver — altitude-based deconfliction for swarm goto commands.

Strategy: assign each drone a unique flight altitude so they share the same
(x, y) target but never occupy the same vertical band simultaneously.

Default: base_z=1.0 m, separation=0.5 m → drone[0]→1.0 m, drone[1]→1.5 m, ...
"""


class ConflictResolver:
    """
    Assigns unique flight altitudes to prevent mid-air collisions when multiple
    drones navigate to the same (x, y) target.
    """

    def __init__(self, base_z: float = 1.0, separation: float = 0.5):
        """
        Args:
            base_z:      Altitude assigned to the first drone (metres).
            separation:  Vertical gap between consecutive drone altitudes (metres).
        """
        self.base_z = base_z
        self.separation = separation

    def assign_altitudes(self, n: int) -> list[float]:
        """
        Return a list of n unique altitudes.

        Example (n=2, base=1.0, sep=0.5) → [1.0, 1.5]
        Example (n=3, base=1.0, sep=0.5) → [1.0, 1.5, 2.0]
        """
        return [self.base_z + i * self.separation for i in range(n)]

    def assign_altitudes_for(
        self, drone_ids: list[str], override_zs: list[float] | None = None
    ) -> dict[str, float]:
        """
        Return a mapping {drone_id: altitude}.

        If *override_zs* is provided (one per drone), use those values directly;
        otherwise auto-assign via base+separation.
        """
        if override_zs and len(override_zs) == len(drone_ids):
            return dict(zip(drone_ids, override_zs))
        altitudes = self.assign_altitudes(len(drone_ids))
        return dict(zip(drone_ids, altitudes))
