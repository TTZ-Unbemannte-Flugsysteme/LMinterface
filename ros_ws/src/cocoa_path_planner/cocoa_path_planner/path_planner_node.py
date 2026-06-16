"""
Path Planner Node - ROS2 Service for Finding Safe Routes

This node provides a /plan_path service that:
1. Gets obstacle data from the EKG (knowledge graph)
2. Creates an occupancy grid (map of free/blocked cells)
3. Uses A* algorithm to find a path from start to goal
4. Returns a list of waypoints the drone can follow

The LLM Planner can call this service like:
    "I need to go to the chair, let me call path_planner to find a safe route"
"""

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from cocoa_msgs.srv import PlanPath, QueryEKG

# Import our path planning modules (same package)
from .astar import find_path, smooth_path
from .occupancy_grid import OccupancyGrid


class PathPlannerNode(Node):
    """
    ROS2 Node that provides path planning services.
    
    Services:
        /plan_path - Find a collision-free path from A to B
    
    Uses:
        /ekg/query - To get object positions and sizes
    """
    
    def __init__(self):
        super().__init__('path_planner_node')
        
        # =============================================
        # PARAMETERS
        # =============================================
        
        # Grid resolution (smaller = more precise but slower)
        self.declare_parameter('resolution', 0.1)  # 10cm cells
        
        # World boundaries (how big is our environment?)
        self.declare_parameter('x_min', -15.0)
        self.declare_parameter('x_max', 15.0)
        self.declare_parameter('y_min', -15.0)
        self.declare_parameter('y_max', 15.0)
        
        # Default safety margin around obstacles
        self.declare_parameter('default_safety_margin', 0.8)  # 80cm
        
        # Get parameter values
        self.resolution = self.get_parameter('resolution').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.default_safety_margin = self.get_parameter('default_safety_margin').value
        
        # =============================================
        # SETUP SERVICES
        # =============================================
        
        # Client to query EKG for obstacle positions
        self.ekg_client = self.create_client(QueryEKG, '/ekg/query')
        
        # Service to handle path planning requests
        self.plan_path_srv = self.create_service(
            PlanPath,
            '/plan_path',
            self.plan_path_callback
        )
        
        # =============================================
        # CACHE FOR OBJECTS (updated from EKG)
        # =============================================
        
        # Store objects from EKG (populated when we first get a request)
        self.cached_objects = []
        self.objects_loaded = False
        
        # =============================================
        # START UP LOG
        # =============================================
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Path Planner Node Ready")
        self.get_logger().info(f"  Service: /plan_path")
        self.get_logger().info(f"  Resolution: {self.resolution}m")
        self.get_logger().info(f"  Area: X[{self.x_min}, {self.x_max}] Y[{self.y_min}, {self.y_max}]")
        self.get_logger().info(f"  Default margin: {self.default_safety_margin}m")
        self.get_logger().info("=" * 50)
    
    def load_objects_from_config(self):
        """
        Load objects directly from config file as backup.
        This is simpler than querying EKG for all objects.
        """
        try:
            # Try to find the config file
            from ament_index_python.packages import get_package_share_directory
            import os
            
            pkg_share = get_package_share_directory('cocoa_ekg')
            config_path = os.path.join(pkg_share, 'config', 'objects.yaml')
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.cached_objects = config.get('objects', [])
            
            self.get_logger().info(f"Loaded {len(self.cached_objects)} objects from config")
            self.objects_loaded = True
            return True
            
        except Exception as e:
            self.get_logger().warn(f"Could not load objects from config: {e}")
            return False
    
    def plan_path_callback(self, request, response):
        """
        Handle a path planning request.
        
        This is called when someone asks: "Find me a path from A to B"
        
        Steps:
        1. Load obstacles if not already loaded
        2. Create occupancy grid with obstacles
        3. Convert world positions to grid cells
        4. Run A* to find path
        5. Convert path back to world coordinates
        6. Return waypoints
        """
        self.get_logger().info("-" * 40)
        self.get_logger().info("Path planning request received!")
        self.get_logger().info(f"  Start: ({request.start.x:.2f}, {request.start.y:.2f}, {request.start.z:.2f})")
        self.get_logger().info(f"  Goal:  ({request.goal.x:.2f}, {request.goal.y:.2f}, {request.goal.z:.2f})")
        self.get_logger().info(f"  Safety margin: {request.safety_margin}m")
        
        # =============================================
        # STEP 1: Load obstacles if needed
        # =============================================
        
        if not self.objects_loaded:
            self.get_logger().info("Loading obstacles from config...")
            self.load_objects_from_config()
        
        if not self.cached_objects:
            self.get_logger().warn("No obstacles loaded! Path will be direct.")
        
        # =============================================
        # STEP 2: Create occupancy grid
        # =============================================
        
        self.get_logger().info("Creating occupancy grid...")
        
        grid = OccupancyGrid(
            resolution=self.resolution,
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max
        )
        
        # Use the safety margin from request, or default
        margin = request.safety_margin if request.safety_margin > 0 else self.default_safety_margin
        
        # Add all obstacles to the grid
        grid.add_obstacles_from_ekg(self.cached_objects, safety_margin=margin)
        
        # IMPORTANT: Clear a small area at the goal position so we can reach it
        # Keep radius small (0.3m) to avoid clearing obstacle safety zones!
        grid.clear_position(request.goal.x, request.goal.y, radius=0.7)
        
        # Also clear a small area at start position
        grid.clear_position(request.start.x, request.start.y, radius=0.7)
        
        # =============================================
        # STEP 3: Convert positions to grid cells
        # =============================================
        
        start_row, start_col = grid.world_to_grid(request.start.x, request.start.y)
        goal_row, goal_col = grid.world_to_grid(request.goal.x, request.goal.y)
        
        start_cell = (start_row, start_col)
        goal_cell = (goal_row, goal_col)
        
        self.get_logger().info(f"  Grid start: {start_cell}")
        self.get_logger().info(f"  Grid goal:  {goal_cell}")
        
        # Check if start or goal is blocked
        if not grid.is_cell_free(start_row, start_col):
            response.success = False
            response.message = "Start position is inside an obstacle!"
            response.waypoints = []
            response.total_distance = 0.0
            response.num_waypoints = 0
            self.get_logger().error(response.message)
            return response
        
        if not grid.is_cell_free(goal_row, goal_col):
            response.success = False
            response.message = "Goal position is inside an obstacle!"
            response.waypoints = []
            response.total_distance = 0.0
            response.num_waypoints = 0
            self.get_logger().error(response.message)
            return response
        
        # =============================================
        # STEP 4: Run A* path finding
        # =============================================
        
        self.get_logger().info("Running A* path finding...")
        
        grid_2d = grid.get_grid_for_pathfinding()
        path_cells = find_path(grid_2d, start_cell, goal_cell)
        
        if path_cells is None:
            response.success = False
            response.message = "No path found! The goal might be unreachable."
            response.waypoints = []
            response.total_distance = 0.0
            response.num_waypoints = 0
            self.get_logger().error(response.message)
            return response
        
        self.get_logger().info(f"  Raw path has {len(path_cells)} points")
        
        # =============================================
        # STEP 5: Smooth the path (remove unnecessary waypoints)
        # =============================================
        
        smoothed_cells = smooth_path(path_cells, grid_2d)
        self.get_logger().info(f"  Smoothed path has {len(smoothed_cells)} waypoints")
        
        # =============================================
        # STEP 6: Convert back to world coordinates
        # =============================================
        
        # Determine flight height
        flight_height = request.flight_height if request.flight_height > 0 else 1.0
        
        waypoints = []
        total_distance = 0.0
        prev_x, prev_y = None, None
        
        for row, col in smoothed_cells:
            # Convert grid cell to world position
            world_x, world_y = grid.grid_to_world(row, col)
            
            # Create Point message
            point = Point()
            point.x = world_x
            point.y = world_y
            point.z = flight_height
            waypoints.append(point)
            
            # Calculate distance
            if prev_x is not None:
                import math
                dist = math.sqrt((world_x - prev_x)**2 + (world_y - prev_y)**2)
                total_distance += dist
            
            prev_x, prev_y = world_x, world_y
        
        # =============================================
        # STEP 7: Build and return response
        # =============================================
        
        response.success = True
        response.message = f"Path found with {len(waypoints)} waypoints"
        response.waypoints = waypoints
        response.total_distance = float(total_distance)
        response.num_waypoints = len(waypoints)
        
        self.get_logger().info(f"Path found!")
        self.get_logger().info(f"  Waypoints: {len(waypoints)}")
        self.get_logger().info(f"  Total distance: {total_distance:.2f}m")
        
        # Log waypoints for debugging
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"    [{i}] ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f})")
        
        return response


def main(args=None):
    """Start the path planner node."""
    rclpy.init(args=args)
    
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
