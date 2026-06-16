"""
Formation module — computes inspection positions around a target object.

Given a target (x, y) and the number of drones, this module queries the EKG
to find the object at that location and distributes drones around its
actual boundary (width × depth) plus a configurable safety margin.

If no EKG object is found at the target, a simple circular fallback
with the inspection_radius is used.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from cocoa_ekg.knowledge_graph import KnowledgeGraph


@dataclass
class InspectionWaypoint:
    """A single drone's inspection position and heading."""
    x: float
    y: float
    yaw: float          # radians — pointing toward the target
    object_name: str    # name of the matched EKG object (or "unknown")


class FormationPlanner:
    """
    Computes drone positions for inspecting a target location.

    Uses the EKG to look up the object's bounding box at the target
    coordinates and distributes drones on an elliptical perimeter
    around it (half-width + margin, half-depth + margin).

    Usage:
        planner = FormationPlanner(inspection_radius=1.5)
        waypoints = planner.compute_inspection_positions(
            target_x=-8.0, target_y=6.0, n_drones=2
        )
        for wp in waypoints:
            print(wp.x, wp.y, wp.yaw, wp.object_name)
    """

    def __init__(
        self,
        inspection_radius: float = 1.5,
        match_threshold: float = 0.5,
        ekg: Optional[KnowledgeGraph] = None,
    ):
        """
        Args:
            inspection_radius: Safety margin added beyond the object boundary (m).
            match_threshold:   Max distance (m) to consider an EKG object as
                               the target at (target_x, target_y).
            ekg:               Pre-loaded KnowledgeGraph. If None, a new one is
                               created from the default config.
        """
        self.inspection_radius = inspection_radius
        self.match_threshold = match_threshold
        self.ekg = ekg if ekg is not None else KnowledgeGraph()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_inspection_positions(
        self,
        target_x: float,
        target_y: float,
        n_drones: int,
    ) -> List[InspectionWaypoint]:
        """
        Return *n_drones* inspection waypoints arranged around the target.

        Each waypoint contains (x, y, yaw, object_name).
        Yaw is set so the drone faces the centre of the target.

        Args:
            target_x: Target X coordinate.
            target_y: Target Y coordinate.
            n_drones: Number of drones to distribute.

        Returns:
            List of InspectionWaypoint, one per drone (same order as drone_ids).
        """
        rx, ry, obj_name = self._resolve_radii(target_x, target_y)

        waypoints: List[InspectionWaypoint] = []
        for i in range(n_drones):
            angle = i * (2 * math.pi / n_drones)
            wp = InspectionWaypoint(
                x=target_x + rx * math.cos(angle),
                y=target_y + ry * math.sin(angle),
                yaw=angle + math.pi,   # face toward target
                object_name=obj_name,
            )
            waypoints.append(wp)

        return waypoints

    def get_object_at(
        self, target_x: float, target_y: float
    ) -> Tuple[Optional[str], Tuple[float, float, float]]:
        """
        Return the EKG object name and size at the given coordinates, or
        (None, (0,0,0)) if nothing matches.
        """
        for obj in self.ekg.get_all_objects():
            dist = math.sqrt(
                (obj.position[0] - target_x) ** 2
                + (obj.position[1] - target_y) ** 2
            )
            if dist < self.match_threshold:
                return obj.name, obj.size
        return None, (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _resolve_radii(
        self, target_x: float, target_y: float
    ) -> Tuple[float, float, str]:
        """
        Look up the EKG for an object at (target_x, target_y).
        Return (rx, ry, object_name).
        """
        obj_name, size = self.get_object_at(target_x, target_y)

        if obj_name is not None:
            # size = (width, depth, height)
            rx = (size[0] / 2.0) + self.inspection_radius
            ry = (size[1] / 2.0) + self.inspection_radius
            return rx, ry, obj_name

        # Fallback: no object found — use a simple circle
        return self.inspection_radius, self.inspection_radius, "unknown"
