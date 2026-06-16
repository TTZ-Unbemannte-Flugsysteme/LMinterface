"""
LGA Node - Language-Guided Abstraction ROS2 Node
Filters context based on intent type, checks collisions with EKG objects
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, PoseStamped

from cocoa_msgs.srv import QueryLGA, QueryEKG, GetAllObjects
from std_srvs.srv import Trigger


class LGANode(Node):
    """
    Language-Guided Abstraction Node
    
    - Provides QueryLGA service
    - Uses EKG to get object positions
    - Checks for collisions based on intent direction
    - Simulates battery level
    """
    
    # Intent -> relevant check mapping
    INTENT_CHECKS = {
        "MOVE_DIRECTION": ["obstacles", "battery"],
        "GO_TO_LOCATION": ["obstacles", "battery", "target_distance"],
        "CHANGE_ALTITUDE": ["ceiling", "floor", "battery"],
        "TAKEOFF": ["clearance", "battery"],
        "LAND": ["landing_zone"],
        "HOVER": ["battery"],
        "ROTATE": [],  # Usually safe
        "EMERGENCY_STOP": [],  # Immediate action
        "QUERY": ["battery", "nearby_objects"],  # For status and environment queries
        "COMPLEX_TASK": ["battery", "all_objects"],  # Needs full environment context
        "UNKNOWN": [],
    }
    
    # Direction to vector mapping
    DIRECTION_VECTORS = {
        "forward": (1.0, 0.0, 0.0),
        "backward": (-1.0, 0.0, 0.0),
        "left": (0.0, 1.0, 0.0),
        "right": (0.0, -1.0, 0.0),
        "up": (0.0, 0.0, 1.0),
        "down": (0.0, 0.0, -1.0),
    }
    
    # Maximum clearance value (instead of float('inf') for serialization)
    MAX_CLEARANCE = 999.0
    
    # Demo objects for obstacle checking (fallback if EKG fetch fails)
    # Should be populated from EKG at startup
    # Format: name -> (x, y, z, size_x, size_y, size_z)
    FALLBACK_OBJECTS = {
        "shelf_a": (-8.0, 6.0, 1.0, 2.5, 0.8, 2.0),
        "shelf_b": (8.0, 6.0, 1.0, 2.5, 0.8, 2.0),
        "pallet_1": (-5.0, 2.0, 0.6, 1.3, 1.0, 1.2),
        "pallet_2": (5.0, 2.0, 0.6, 1.3, 1.0, 1.2),
        "center_rack": (0.0, 3.0, 0.5, 0.5, 2.0, 1.0),
        "forklift": (5.0, -5.0, 0.6, 1.5, 1.0, 1.2),
        "box_1": (-5.0, -5.0, 0.4, 0.5, 0.5, 0.5),
        "landing_pad": (0.0, -8.0, 0.01, 1.6, 1.6, 0.02),
    }
    
    def __init__(self):
        super().__init__('lga_node')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('drone_id', 'cf231')
        self.drone_id = self.get_parameter('drone_id').value
        
        # Height threshold to determine if drone is flying
        self.FLYING_HEIGHT_THRESHOLD = 0.1  # meters
        
        # Simulated state
        self.battery_level = 100.0  # Start full
        
        # Drone state from pose (single source of truth)
        self.drone_position = Point(x=0.0, y=0.0, z=0.0)
        self.is_flying = False
        self.drone_state = "grounded"  # grounded, hovering, flying
        self.drone_state = "grounded"  # grounded, hovering, flying
        self.drone_yaw = 0.0  # Heading in degrees
        self.pose_received = False  # Track if we've received any pose yet
        
        # Subscribe to drone pose for state tracking
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_id}/pose',
            self._pose_callback,
            10
        )
        
        # Create EKG client to query object positions
        self.ekg_client = self.create_client(
            QueryEKG, '/ekg/query', 
            callback_group=self.cb_group
        )
        
        # Create EKG client to get all objects for collision checking
        self.ekg_objects_client = self.create_client(
            GetAllObjects, '/ekg/get_all_objects',
            callback_group=self.cb_group
        )
        
        # Create QueryLGA service
        self.lga_srv = self.create_service(
            QueryLGA, '/lga/query',
            self.query_lga_callback,
            callback_group=self.cb_group
        )
        
        # Timer to simulate battery drain
        self.battery_timer = self.create_timer(10.0, self._drain_battery)
        
        # Cache of known objects from EKG (for collision checking)
        self.known_objects = {}
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("LGA Node Ready")
        self.get_logger().info("  Service: /lga/query")
        self.get_logger().info(f"  Drone: {self.drone_id}")
        self.get_logger().info(f"  Pose topic: /{self.drone_id}/pose")
        self.get_logger().info(f"  Battery: {self.battery_level}%")
        self.get_logger().info("=" * 50)
        
        # Reset service (for test runner to reset battery between sequences)
        self.reset_srv = self.create_service(
            Trigger, '/lga/reset',
            self._reset_callback,
            callback_group=self.cb_group
        )
        
        # Fetch objects from EKG at startup
        self._fetch_objects_from_ekg()
    
    def _pose_callback(self, msg: PoseStamped):
        """Update drone state from pose - single source of truth"""
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info(f"First pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
        
        
        self.drone_position.x = msg.pose.position.x
        self.drone_position.y = msg.pose.position.y
        self.drone_position.z = msg.pose.position.z
        
        # calculate yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        
        # Determine flying state from height
        z = msg.pose.position.z
        self.is_flying = z > self.FLYING_HEIGHT_THRESHOLD
        
        if not self.is_flying:
            self.drone_state = "grounded"
        else:
            self.drone_state = "flying"

    def _drain_battery(self):
        """Simulate battery drain over time"""
        self.battery_level = max(0.0, self.battery_level - random.uniform(0.5, 1.5))
        if self.battery_level < 20.0:
            self.get_logger().warn(f"Low battery: {self.battery_level:.1f}%")
    
    def _reset_callback(self, request, response):
        """Reset LGA state (battery, etc.) between test sequences."""
        self.battery_level = 100.0
        self.get_logger().info('[RESET] Battery reset to 100%')
        response.success = True
        response.message = 'Battery reset to 100%'
        return response

    def _fetch_objects_from_ekg(self):
        """Fetch all objects from EKG for collision checking"""
        self.get_logger().info("Fetching objects from EKG for collision checking...")
        
        # Wait for service to be available
        if not self.ekg_objects_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "EKG get_all_objects service not available, using fallback objects"
            )
            self.known_objects = dict(self.FALLBACK_OBJECTS)
            return
        
        request = GetAllObjects.Request()
        future = self.ekg_objects_client.call_async(request)
        
        # Use spin_until_future_complete since we're called from __init__ 
        # and the executor may not be running yet
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error("EKG get_all_objects timed out, using fallback objects")
            self.known_objects = dict(self.FALLBACK_OBJECTS)
            return
        
        result = future.result()
        if result is not None and result.count > 0:
            # Populate known_objects from EKG response (now with sizes)
            self.known_objects = {}
            for i in range(result.count):
                name = result.names[i]
                pos = result.positions[i]
                # Get sizes if available, otherwise use defaults
                if hasattr(result, 'sizes') and i < len(result.sizes):
                    size = result.sizes[i]
                    self.known_objects[name] = (pos.x, pos.y, pos.z, size.x, size.y, size.z)
                else:
                    # Fallback to default size if service doesn't provide sizes
                    self.known_objects[name] = (pos.x, pos.y, pos.z, 0.5, 0.5, 0.5)
            
            self.get_logger().info(f"Loaded {len(self.known_objects)} objects from EKG:")
            for name, data in self.known_objects.items():
                self.get_logger().info(f"  - {name}: pos=({data[0]:.1f}, {data[1]:.1f}, {data[2]:.1f}), size=({data[3]:.1f}, {data[4]:.1f}, {data[5]:.1f})")
        else:
            self.get_logger().warn("EKG returned no objects, using fallback objects")
            self.known_objects = dict(self.FALLBACK_OBJECTS)

    def query_lga_callback(self, request, response):
        """
        Handle QueryLGA service requests.
        
        THESIS A/B COMPARISON:
        - baseline_mode=True:  Return ALL context (no filtering)
        - baseline_mode=False: Return filtered context based on intent
        """
        mode = "BASELINE" if request.baseline_mode else "FILTERED"
        self.get_logger().info(
            f'LGA query [{mode}]: intent={request.intent_type}, '
            f'target={request.target_name}, direction={request.direction}'
        )
        
        # Initialize response fields
        response.safe = True
        response.target_found = False
        response.target_position = Point()
        response.drone_position = self.drone_position
        response.clearance = self.MAX_CLEARANCE
        response.obstacles = []
        response.battery_level = self.battery_level
        response.drone_state = self.drone_state
        response.is_flying = self.is_flying
        response.warning = ""
        
        warnings = []
        context_parts = []
        
        # ============================================================
        # BASELINE MODE: Return ALL context (for A/B comparison)
        # ============================================================
        if request.baseline_mode:
            # 1. Drone state (always)
            # 1. Drone state (always)
            context_parts.append(f"drone_pos: ({self.drone_position.x:.2f}, {self.drone_position.y:.2f}, {self.drone_position.z:.2f})")
            context_parts.append(f"drone_yaw: {self.drone_yaw:.1f}")
            context_parts.append(f"is_flying: {self.is_flying}")
            context_parts.append(f"drone_state: {self.drone_state}")
            
            # 2. Battery (always in baseline)
            context_parts.append(f"battery: {self.battery_level:.0f}%")
            if self.battery_level < 10.0:
                response.safe = False
                warnings.append("CRITICAL: Battery below 10%!")
            
            # 3. ALL objects from EKG (baseline - no filtering)
            context_parts.append("ENVIRONMENT OBJECTS:")
            for obj_name, obj_data in self.known_objects.items():
                if len(obj_data) >= 6:
                     context_parts.append(f"  {obj_name}: pos=({obj_data[0]:.1f}, {obj_data[1]:.1f}, {obj_data[2]:.1f}), size=({obj_data[3]:.1f}, {obj_data[4]:.1f}, {obj_data[5]:.1f})")
                else:
                     context_parts.append(f"  {obj_name}: ({obj_data[0]:.1f}, {obj_data[1]:.1f}, {obj_data[2]:.1f})")
            
            # 4. If target specified, still look it up
            if request.target_name:
                target_pos = self._query_ekg_for_target(request.target_name)
                if target_pos:
                    response.target_found = True
                    response.target_position = target_pos
            
            # 5. All obstacles info (no filtering)
            if self.known_objects:
                response.obstacles = list(self.known_objects.keys())
            
            if warnings:
                context_parts.extend(warnings)
            
            response.warning = "; ".join(warnings) if warnings else ""
            response.filtered_context = "; ".join(context_parts)
            response.context_token_count = len(response.filtered_context) // 4
            
            self.get_logger().info(
                f'LGA BASELINE: {response.context_token_count} tokens (all objects)'
            )
            return response
        
        # ============================================================
        # FILTERED MODE: Return only what's needed for this intent
        # ============================================================
        checks = self.INTENT_CHECKS.get(request.intent_type, [])
        
        # Drone state (always needed by Planner)
        # Drone state (always needed by Planner)
        context_parts.append(f"drone_pos: ({self.drone_position.x:.2f}, {self.drone_position.y:.2f}, {self.drone_position.z:.2f})")
        context_parts.append(f"drone_yaw: {self.drone_yaw:.1f}")
        context_parts.append(f"is_flying: {self.is_flying}")
        context_parts.append(f"drone_state: {self.drone_state}")
        
        # Battery check (only if relevant to this intent)
        if "battery" in checks:
            context_parts.append(f"battery: {self.battery_level:.0f}%")
            if self.battery_level < 10.0:
                response.safe = False
                warnings.append("CRITICAL: Battery below 10%!")
            elif self.battery_level < 20.0:
                warnings.append("WARNING: Low battery")
        
        # GO_TO_LOCATION: Only target object
        if request.intent_type == "GO_TO_LOCATION" and request.target_name:
            target_pos = self._query_ekg_for_target(request.target_name)
            if target_pos is not None:
                response.target_found = True
                response.target_position = target_pos
                # Include target position AND current flight height (use drone z, not object z)
                # This prevents the planner from trying to fly to ground-level objects like landing_pad
                flight_height = max(self.drone_position.z, 1.0)  # Minimum 1.0m flight height
                context_parts.append(f"{request.target_name} at ({target_pos.x:.2f}, {target_pos.y:.2f}); flight_height: {flight_height:.2f}")
                obstacles, min_clearance, closest = self._check_obstacles_to_point(
                    request.current_position, target_pos
                )
                response.obstacles = obstacles
                response.clearance = min_clearance
                if obstacles:
                    warnings.append(f"INFO: {len(obstacles)} obstacle(s) in path")
            else:
                warnings.append(f"Target '{request.target_name}' not found in EKG")
        
        # MOVE_DIRECTION: Obstacles and clearance in that direction
        elif "obstacles" in checks and request.direction:
            obstacles, min_clearance, closest_obstacle = self._check_obstacles_in_direction(
                request.current_position, request.direction, request.distance
            )
            response.obstacles = obstacles
            response.clearance = min_clearance
            
            # Always include clearance info for MOVE_DIRECTION
            if obstacles:
                context_parts.append(f"obstacles in {request.direction}: {', '.join(obstacles)}")
                context_parts.append(f"clearance: {min_clearance:.2f}m")
            else:
                context_parts.append(f"clear path {request.direction} for {request.distance:.1f}m")
            
            if min_clearance < request.distance:
                response.safe = False
                warnings.append(f"Obstacle '{closest_obstacle}' at {min_clearance:.2f}m")
        
        # Landing zone check
        if "landing_zone" in checks:
            response.clearance = request.current_position.z
        
        # Nearby objects check (for QUERY intents)
        if "nearby_objects" in checks:
            nearby_radius = 5.0  # meters
            nearby = []
            for obj_name, obj_pos in self.known_objects.items():
                dist = math.sqrt(
                    (obj_pos[0] - self.drone_position.x) ** 2 +
                    (obj_pos[1] - self.drone_position.y) ** 2 +
                    (obj_pos[2] - self.drone_position.z) ** 2
                )
                if dist <= nearby_radius:
                    nearby.append(f"{obj_name} ({dist:.1f}m)")
            
            if nearby:
                context_parts.append(f"nearby objects: {', '.join(nearby)}")
            else:
                context_parts.append("no objects within 5m")
        
        # All objects check (for COMPLEX_TASK)
        if "all_objects" in checks:
            context_parts.append("ENVIRONMENT OBJECTS (name: pos, size):")
            for obj_name, obj_data in self.known_objects.items():
                if len(obj_data) >= 6:
                     context_parts.append(f"  {obj_name}: pos=({obj_data[0]:.1f}, {obj_data[1]:.1f}, {obj_data[2]:.1f}), size=({obj_data[3]:.1f}, {obj_data[4]:.1f}, {obj_data[5]:.1f})")
                else:
                     context_parts.append(f"  {obj_name}: pos=({obj_data[0]:.1f}, {obj_data[1]:.1f}, {obj_data[2]:.1f}), size=(0.5, 0.5, 0.5)")
        
        if warnings:
            context_parts.extend(warnings)
        
        if not response.safe:
            context_parts.append("ACTION NOT SAFE")
        
        response.warning = "; ".join(warnings) if warnings else ""
        response.filtered_context = "; ".join(context_parts)
        response.context_token_count = len(response.filtered_context) // 4
        
        self.get_logger().info(
            f'LGA FILTERED: {response.context_token_count} tokens'
        )
        
        return response
    
    def _query_ekg_for_target(self, target_name: str):
        """Query EKG to get target position. Returns Point or None."""
        import time
        
        if not self.ekg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("EKG service not available")
            return None
        
        request = QueryEKG.Request()
        request.object_name = target_name
        
        future = self.ekg_client.call_async(request)
        
        # Wait for response (polling)
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.05)
        
        if not future.done():
            self.get_logger().error("EKG service call timed out")
            return None
        
        result = future.result()
        if result is not None and result.found:
            return result.position
        return None
    
    def _check_obstacles_to_point(self, current_pos, target_pos):
        """Check for obstacles on path to a specific target point."""
        import math
        
        obstacles_in_path = []
        min_clearance = self.MAX_CLEARANCE
        closest_obstacle = None
        
        for obj_name, obj_pos in self.known_objects.items():
            # Skip if this is the target
            obj_distance = self._point_to_line_distance(
                current_pos, (target_pos.x, target_pos.y, target_pos.z), obj_pos
            )
            
            direct_distance = math.sqrt(
                (obj_pos[0] - current_pos.x) ** 2 +
                (obj_pos[1] - current_pos.y) ** 2 +
                (obj_pos[2] - current_pos.z) ** 2
            )
            
            target_distance = math.sqrt(
                (target_pos.x - current_pos.x) ** 2 +
                (target_pos.y - current_pos.y) ** 2 +
                (target_pos.z - current_pos.z) ** 2
            )
            
            # If object is close to path and between us and target
            if obj_distance < 0.5 and direct_distance < target_distance:
                obstacles_in_path.append(obj_name)
                if direct_distance < min_clearance:
                    min_clearance = direct_distance
                    closest_obstacle = obj_name
        
        if not obstacles_in_path:
            min_clearance = self.MAX_CLEARANCE
        
        return obstacles_in_path, min_clearance, closest_obstacle

    def _check_obstacles_in_direction(self, current_pos, direction, distance):
        """
        Check for obstacles in the specified direction using AABB collision.
        Uses cached object positions and sizes from EKG.
        
        Objects are stored as (x, y, z, size_x, size_y, size_z) tuples.
        """
        vector = self.DIRECTION_VECTORS.get(direction, (1.0, 0.0, 0.0))
        
        # Safety margin around objects (reduced from 0.5m for real-world indoor testing)
        SAFETY_MARGIN = 0.2  # meters
        
        # Calculate endpoint of intended movement
        end_x = current_pos.x + vector[0] * distance
        end_y = current_pos.y + vector[1] * distance
        end_z = current_pos.z + vector[2] * distance
        
        obstacles_in_path = []
        min_clearance = self.MAX_CLEARANCE
        closest_obstacle = None
        
        for obj_name, obj_data in self.known_objects.items():
            # Extract position and size (obj_data is now 6-tuple: x, y, z, size_x, size_y, size_z)
            obj_x, obj_y, obj_z = obj_data[0], obj_data[1], obj_data[2]
            if len(obj_data) >= 6:
                size_x, size_y, size_z = obj_data[3], obj_data[4], obj_data[5]
            else:
                # Fallback for old format (position-only)
                size_x, size_y, size_z = 0.5, 0.5, 0.5
            
            # Calculate AABB (Axis-Aligned Bounding Box) with safety margin
            half_x = size_x / 2.0 + SAFETY_MARGIN
            half_y = size_y / 2.0 + SAFETY_MARGIN
            half_z = size_z / 2.0 + SAFETY_MARGIN
            
            box_min_x = obj_x - half_x
            box_max_x = obj_x + half_x
            box_min_y = obj_y - half_y
            box_max_y = obj_y + half_y
            box_min_z = obj_z - half_z
            box_max_z = obj_z + half_z
            
            # Check if line segment from current_pos to end intersects with AABB
            # Simple check: is any point along the path inside the expanded AABB?
            # Use parametric line: point(t) = current_pos + t * (end - current)
            
            # Calculate intersection times for each axis
            dx = end_x - current_pos.x
            dy = end_y - current_pos.y
            dz = end_z - current_pos.z
            
            t_enter = 0.0
            t_exit = 1.0
            
            # Check X axis
            if abs(dx) > 1e-6:
                t1 = (box_min_x - current_pos.x) / dx
                t2 = (box_max_x - current_pos.x) / dx
                if t1 > t2:
                    t1, t2 = t2, t1
                t_enter = max(t_enter, t1)
                t_exit = min(t_exit, t2)
            elif current_pos.x < box_min_x or current_pos.x > box_max_x:
                continue  # Line parallel to X and outside box
            
            # Check Y axis
            if abs(dy) > 1e-6:
                t1 = (box_min_y - current_pos.y) / dy
                t2 = (box_max_y - current_pos.y) / dy
                if t1 > t2:
                    t1, t2 = t2, t1
                t_enter = max(t_enter, t1)
                t_exit = min(t_exit, t2)
            elif current_pos.y < box_min_y or current_pos.y > box_max_y:
                continue  # Line parallel to Y and outside box
            
            # Check Z axis
            if abs(dz) > 1e-6:
                t1 = (box_min_z - current_pos.z) / dz
                t2 = (box_max_z - current_pos.z) / dz
                if t1 > t2:
                    t1, t2 = t2, t1
                t_enter = max(t_enter, t1)
                t_exit = min(t_exit, t2)
            elif current_pos.z < box_min_z or current_pos.z > box_max_z:
                continue  # Line parallel to Z and outside box
            
            # If t_enter <= t_exit and within [0, 1], line intersects the box
            if t_enter <= t_exit and t_exit >= 0 and t_enter <= 1:
                obstacles_in_path.append(obj_name)
                
                # Calculate clearance as distance to first intersection point
                intersection_dist = t_enter * distance
                if intersection_dist < min_clearance:
                    min_clearance = max(0, intersection_dist)
                    closest_obstacle = obj_name
        
        if not obstacles_in_path:
            min_clearance = distance  # Clear path
        
        return obstacles_in_path, min_clearance, closest_obstacle

    def _point_to_line_distance(self, line_start, line_end, point):
        """Calculate distance from a point to a line segment"""
        # Vector from start to end
        dx = line_end[0] - line_start.x
        dy = line_end[1] - line_start.y
        dz = line_end[2] - line_start.z
        
        # Vector from start to point
        px = point[0] - line_start.x
        py = point[1] - line_start.y
        pz = point[2] - line_start.z
        
        # Project point onto line
        line_len_sq = dx*dx + dy*dy + dz*dz
        if line_len_sq == 0:
            return math.sqrt(px*px + py*py + pz*pz)
        
        t = max(0, min(1, (px*dx + py*dy + pz*dz) / line_len_sq))
        
        # Closest point on line
        closest_x = line_start.x + t * dx
        closest_y = line_start.y + t * dy
        closest_z = line_start.z + t * dz
        
        return math.sqrt(
            (point[0] - closest_x) ** 2 +
            (point[1] - closest_y) ** 2 +
            (point[2] - closest_z) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = LGANode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
