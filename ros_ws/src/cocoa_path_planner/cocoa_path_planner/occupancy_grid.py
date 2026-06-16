"""
Occupancy Grid - Converting the World to a Grid Map

This file creates a 2D grid from the objects in EKG (the knowledge graph).
Think of it like taking a bird's-eye photo of the room and dividing it into squares.

Each square (cell) is either:
- FREE (0) = The drone can fly here
- BLOCKED (1) = There's an obstacle here

Example:
    Real world (10m x 10m room):
    
    [table at (5,0)]     [lamp at (2,-3)]
                  
         [wall at (2,2)]
                  
         [drone]
    
    Grid (100x100 cells, 0.1m each):
    
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 1 1 0 0 0 0 0 0  <- wall obstacle
    0 0 1 1 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    ...
"""

import numpy as np


class OccupancyGrid:
    """
    A class that creates and manages a 2D grid map of obstacles.
    
    The grid is like a game board where:
    - Each cell represents a small area (default 10cm x 10cm)
    - Obstacles from EKG are marked as blocked cells
    - A safety margin is added around obstacles so the drone doesn't get too close
    """
    
    def __init__(self, resolution=0.1, x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0):
        """
        Create a new occupancy grid.
        
        Args:
            resolution: Size of each cell in meters (0.1 = 10cm)
            x_min, x_max: How far the grid extends in X direction
            y_min, y_max: How far the grid extends in Y direction
        """
        # Store settings
        self.resolution = resolution  # Meters per cell
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        
        # Calculate grid size
        # Example: 20m range with 0.1m resolution = 200 cells
        self.width = int((x_max - x_min) / resolution)
        self.height = int((y_max - y_min) / resolution)
        
        # Create empty grid (all zeros = all free)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        
        print(f"Created grid: {self.width} x {self.height} cells")
        print(f"  Resolution: {resolution}m per cell")
        print(f"  Covers: X[{x_min}, {x_max}], Y[{y_min}, {y_max}]")
    
    def world_to_grid(self, x, y):
        """
        Convert real-world coordinates (meters) to grid coordinates (cell indices).
        
        Example:
            World: (1.5, 2.3) meters
            Grid: (15, 23) cells (if resolution is 0.1m)
        """
        # Shift so minimum is at index 0, then divide by resolution
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        
        return row, col
    
    def grid_to_world(self, row, col):
        """
        Convert grid coordinates (cell indices) to real-world coordinates (meters).
        Returns the CENTER of the cell.
        """
        x = self.x_min + (col + 0.5) * self.resolution
        y = self.y_min + (row + 0.5) * self.resolution
        
        return x, y
    
    def is_inside_grid(self, row, col):
        """Check if a grid cell is within the grid boundaries."""
        return 0 <= row < self.height and 0 <= col < self.width
    
    def add_obstacle(self, center_x, center_y, size_x, size_y, safety_margin=0.3):
        """
        Add an obstacle to the grid.
        
        Marks all cells covered by the obstacle (plus safety margin) as BLOCKED.
        
        Args:
            center_x, center_y: Center of the obstacle in meters
            size_x, size_y: Width and depth of obstacle in meters
            safety_margin: Extra space around obstacle (default 30cm)
        
        Example:
            Table at (5, 0) with size (1, 0.6) and margin 0.3
            Will block cells from (4.2, -0.6) to (5.8, 0.6)
        """
        # Calculate obstacle boundaries (with safety margin)
        half_x = (size_x / 2.0) + safety_margin
        half_y = (size_y / 2.0) + safety_margin
        
        # Convert to grid coordinates
        min_row, min_col = self.world_to_grid(center_x - half_x, center_y - half_y)
        max_row, max_col = self.world_to_grid(center_x + half_x, center_y + half_y)
        
        # Mark cells as blocked
        blocked_count = 0
        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                if self.is_inside_grid(row, col):
                    self.grid[row, col] = 1  # 1 = blocked
                    blocked_count += 1
        
        print(f"  Added obstacle at ({center_x}, {center_y}), blocked {blocked_count} cells")
    
    def add_obstacles_from_ekg(self, objects, safety_margin=0.3):
        """
        Add all obstacles from EKG objects list.
        
        Args:
            objects: List of objects, each with:
                     - 'position': [x, y, z]
                     - 'size': [width, depth, height]
                     - 'category': 'obstacle', 'furniture', etc.
            safety_margin: Extra space around each obstacle
        """
        print(f"\nAdding {len(objects)} objects to grid:")
        
        for obj in objects:
            name = obj.get('name', 'unknown')
            position = obj.get('position', [0, 0, 0])
            size = obj.get('size', [0.5, 0.5, 0.5])  # Default size
            category = obj.get('category', 'object')
            
            # Skip very small objects or specific categories if needed
            if category in ['landmark']:  # Landing pads don't block flight
                print(f"  Skipping {name} (category: {category})")
                continue
            
            self.add_obstacle(
                center_x=position[0],
                center_y=position[1],
                size_x=size[0],
                size_y=size[1],
                safety_margin=safety_margin
            )
    
    def is_cell_free(self, row, col):
        """Check if a specific cell is free to move into."""
        if not self.is_inside_grid(row, col):
            return False  # Treat outside grid as blocked
        
        return self.grid[row, col] == 0
    
    def clear_position(self, x, y, radius=0.2):
        """
        Clear (unblock) cells around a specific position.
        
        Use this to make the GOAL position reachable after adding obstacles.
        The target object itself shouldn't block the drone from reaching it!
        
        Args:
            x, y: World coordinates to clear
            radius: Radius in meters to clear (default 20cm)
        """
        # Clear cells in a small radius around the target
        steps = int(radius / self.resolution) + 1
        center_row, center_col = self.world_to_grid(x, y)
        
        cleared = 0
        for dr in range(-steps, steps + 1):
            for dc in range(-steps, steps + 1):
                row = center_row + dr
                col = center_col + dc
                if self.is_inside_grid(row, col):
                    if self.grid[row, col] == 1:
                        self.grid[row, col] = 0  # Mark as free
                        cleared += 1
        
        if cleared > 0:
            print(f"  Cleared {cleared} cells around goal ({x:.2f}, {y:.2f})")
    
    def is_position_free(self, x, y):
        """Check if a world position (in meters) is free."""
        row, col = self.world_to_grid(x, y)
        return self.is_cell_free(row, col)
    
    def get_grid_for_pathfinding(self):
        """
        Get the grid as a 2D list (for the A* algorithm).
        """
        return self.grid.tolist()
    
    def visualize_ascii(self, path=None, start=None, goal=None):
        """
        Print a simple ASCII visualization of the grid.
        
        Symbols:
        - '.' = free space
        - '#' = obstacle
        - 'S' = start
        - 'G' = goal
        - '*' = path
        
        Note: Only shows a portion of the grid (around center) for readability.
        """
        # Show center portion of grid
        center_row = self.height // 2
        center_col = self.width // 2
        view_size = 20  # Show 20x20 area
        
        print("\nGrid visualization (center area):")
        print("  '.' = free, '#' = blocked, 'S'=start, 'G'=goal, '*'=path")
        
        for row in range(center_row - view_size, center_row + view_size):
            line = ""
            for col in range(center_col - view_size, center_col + view_size):
                if not self.is_inside_grid(row, col):
                    line += " "
                    continue
                
                cell = (row, col)
                
                if start and cell == start:
                    line += "S"
                elif goal and cell == goal:
                    line += "G"
                elif path and cell in path:
                    line += "*"
                elif self.grid[row, col] == 1:
                    line += "#"
                else:
                    line += "."
            
            print(line)


# ============================================
# EXAMPLE USAGE (for testing)
# ============================================

if __name__ == "__main__":
    # Create a small grid for testing
    grid = OccupancyGrid(
        resolution=0.5,  # 50cm cells for easier visualization
        x_min=-5, x_max=10,
        y_min=-5, y_max=10
    )
    
    # Sample objects (like from EKG)
    sample_objects = [
        {
            'name': 'table',
            'position': [5.0, 0.0, 0.0],
            'size': [1.0, 0.6, 0.8],
            'category': 'furniture'
        },
        {
            'name': 'wall_obstacle',
            'position': [2.0, 2.0, 0.5],
            'size': [0.2, 3.0, 1.0],
            'category': 'obstacle'
        },
        {
            'name': 'landing_pad',  # Should be skipped
            'position': [-3.0, 0.0, 0.01],
            'size': [1.0, 1.0, 0.02],
            'category': 'landmark'
        },
    ]
    
    # Add obstacles
    grid.add_obstacles_from_ekg(sample_objects, safety_margin=0.3)
    
    # Visualize
    grid.visualize_ascii()
    
    # Test conversion
    test_x, test_y = 2.0, 4.0
    row, col = grid.world_to_grid(test_x, test_y)
    print(f"\nWorld ({test_x}, {test_y}) -> Grid ({row}, {col})")
    print(f"Is free: {grid.is_cell_free(row, col)}")
