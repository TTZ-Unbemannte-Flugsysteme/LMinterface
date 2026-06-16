"""
A* Path Planning Algorithm - Simple Implementation

This file contains a beginner-friendly A* algorithm for finding
the shortest path from start to goal while avoiding obstacles.

Key concepts:
- We use a GRID to represent the world (like a chessboard)
- Each cell is either FREE (0) or BLOCKED (1)
- A* finds the shortest path by exploring cells smartly
"""

import math
import heapq  # For priority queue (always gives us the smallest item first)


def find_path(grid, start, goal):
    """
    Find the shortest path from start to goal using A* algorithm.
    
    How A* works (simple explanation):
    1. Start at the beginning
    2. Look at all neighbors we can move to
    3. Pick the neighbor that looks most promising (closest to goal + shortest path so far)
    4. Repeat until we reach the goal
    
    Args:
        grid: 2D list where 0 = free space, 1 = obstacle
        start: tuple (row, col) - where we begin
        goal: tuple (row, col) - where we want to go
    
    Returns:
        List of (row, col) positions from start to goal, or None if no path exists
    """
    
    # Get grid size
    num_rows = len(grid)
    num_cols = len(grid[0])
    
    # Check if start or goal is blocked
    if grid[start[0]][start[1]] == 1:
        print("Error: Start position is blocked!")
        return None
    
    if grid[goal[0]][goal[1]] == 1:
        print("Error: Goal position is blocked!")
        return None
    
    # ========================================
    # STEP 1: Set up our data structures
    # ========================================
    
    # Priority queue: stores (f_cost, row, col)
    # f_cost = how promising this cell is (lower = better)
    open_list = []
    heapq.heappush(open_list, (0, start[0], start[1]))
    
    # Keep track of where we came from (to reconstruct path later)
    came_from = {}
    
    # g_cost: actual distance traveled from start to this cell
    g_cost = {}
    g_cost[start] = 0
    
    # Keep track of cells we've already fully explored
    closed_set = set()
    
    # ========================================
    # STEP 2: Define how we can move
    # ========================================
    
    # 8 directions: up, down, left, right, and 4 diagonals
    # Each direction is (row_change, col_change, cost)
    # Diagonal moves cost more (sqrt(2) ≈ 1.41)
    directions = [
        (-1, 0, 1.0),   # Up
        (1, 0, 1.0),    # Down
        (0, -1, 1.0),   # Left
        (0, 1, 1.0),    # Right
        (-1, -1, 1.41), # Up-Left (diagonal)
        (-1, 1, 1.41),  # Up-Right (diagonal)
        (1, -1, 1.41),  # Down-Left (diagonal)
        (1, 1, 1.41),   # Down-Right (diagonal)
    ]
    
    # ========================================
    # STEP 3: Main loop - explore until we find the goal
    # ========================================
    
    while len(open_list) > 0:
        # Get the most promising cell (lowest f_cost)
        f, current_row, current_col = heapq.heappop(open_list)
        current = (current_row, current_col)
        
        # Check if we reached the goal!
        if current == goal:
            # We found it! Build the path by going backwards
            path = rebuild_path(came_from, current)
            return path
        
        # Skip if we already explored this cell
        if current in closed_set:
            continue
        
        # Mark as explored
        closed_set.add(current)
        
        # ========================================
        # STEP 4: Look at all neighboring cells
        # ========================================
        
        for row_change, col_change, move_cost in directions:
            # Calculate neighbor position
            neighbor_row = current_row + row_change
            neighbor_col = current_col + col_change
            neighbor = (neighbor_row, neighbor_col)
            
            # Check if neighbor is inside the grid
            if neighbor_row < 0 or neighbor_row >= num_rows:
                continue  # Outside grid
            if neighbor_col < 0 or neighbor_col >= num_cols:
                continue  # Outside grid
            
            # Check if neighbor is an obstacle
            if grid[neighbor_row][neighbor_col] == 1:
                continue  # Can't go through walls!
            
            # Check if we already explored this cell
            if neighbor in closed_set:
                continue
            
            # ========================================
            # STEP 5: Calculate costs for this neighbor
            # ========================================
            
            # g_cost = distance traveled so far + cost to move to neighbor
            new_g_cost = g_cost[current] + move_cost
            
            # Only update if this is a better path to the neighbor
            if neighbor not in g_cost or new_g_cost < g_cost[neighbor]:
                # This is the best path to neighbor so far!
                came_from[neighbor] = current
                g_cost[neighbor] = new_g_cost
                
                # h_cost = estimated distance to goal (straight line)
                h_cost = calculate_distance(neighbor, goal)
                
                # f_cost = g_cost + h_cost (total estimated cost)
                f_cost = new_g_cost + h_cost
                
                # Add to open list
                heapq.heappush(open_list, (f_cost, neighbor_row, neighbor_col))
    
    # If we get here, there's no path!
    print("No path found from start to goal")
    return None


def calculate_distance(point1, point2):
    """
    Calculate straight-line distance between two points.
    This is our "heuristic" - a guess of how far we still need to go.
    
    Uses Euclidean distance (like measuring with a ruler).
    """
    row_diff = point1[0] - point2[0]
    col_diff = point1[1] - point2[1]
    
    distance = math.sqrt(row_diff * row_diff + col_diff * col_diff)
    return distance


def rebuild_path(came_from, current):
    """
    Build the path by working backwards from goal to start.
    
    came_from is like a breadcrumb trail - for each cell,
    it tells us which cell we came from.
    """
    path = [current]
    
    # Keep going backwards until we reach the start
    while current in came_from:
        current = came_from[current]
        path.append(current)
    
    # Reverse so path goes from start to goal
    path.reverse()
    
    return path


def smooth_path(path, grid=None, max_step_cells=20):
    """
    Make the path smoother by removing unnecessary waypoints.
    
    If we can go directly from point A to point C without hitting
    any obstacle, we don't need point B in between.
    
    Args:
        path: List of (row, col) waypoints
        grid: Optional 2D grid for obstacle checking (0=free, 1=blocked)
              If not provided, only removes collinear points (less safe!)
        max_step_cells: Maximum number of grid cells between waypoints (default: 20)
                        At 0.1m resolution, 20 cells = 2m max per step
    """
    if path is None or len(path) <= 2:
        return path
    
    # If no grid provided, use simple collinear check (original behavior)
    if grid is None:
        return _smooth_collinear_only(path)
    
    # With grid, do proper line-of-sight smoothing
    smoothed = [path[0]]  # Always keep start
    
    i = 0
    while i < len(path) - 1:
        # Try to skip points while maintaining clear line of sight
        # But limit maximum distance between waypoints
        j = len(path) - 1
        while j > i + 1:
            # Check distance between path[i] and path[j]
            dr = abs(path[j][0] - path[i][0])
            dc = abs(path[j][1] - path[i][1])
            distance = (dr * dr + dc * dc) ** 0.5
            
            # Only consider if within max step distance AND has clear line of sight
            if distance <= max_step_cells and has_clear_line_of_sight(grid, path[i], path[j]):
                # Can go directly from path[i] to path[j]
                break
            j -= 1
        
        # path[j] is the farthest point we can reach directly (within limits)
        if j > i + 1:
            # We skipped some points!
            smoothed.append(path[j])
            i = j
        else:
            # Can't skip, add next point
            smoothed.append(path[i + 1])
            i += 1
    
    return smoothed


def has_clear_line_of_sight(grid, p1, p2):
    """
    Check if there's a clear line of sight between two grid cells.
    Uses Bresenham's line algorithm to check all cells along the line.
    
    Returns True if all cells along the line are free (0).
    """
    r1, c1 = p1
    r2, c2 = p2
    
    num_rows = len(grid)
    num_cols = len(grid[0]) if num_rows > 0 else 0
    
    # Bresenham's line algorithm
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1
    sc = 1 if c1 < c2 else -1
    err = dr - dc
    
    r, c = r1, c1
    
    while True:
        # Check if current cell is blocked
        if 0 <= r < num_rows and 0 <= c < num_cols:
            if grid[r][c] == 1:
                return False  # Hit an obstacle!
        else:
            return False  # Out of bounds
        
        if r == r2 and c == c2:
            break  # Reached destination
        
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    
    return True  # All cells along the line are free


def _smooth_collinear_only(path):
    """
    Original simple smoothing - only removes collinear points.
    WARNING: This doesn't check for obstacles!
    """
    if path is None or len(path) <= 2:
        return path
    
    smoothed = [path[0]]  # Always keep start
    
    for i in range(1, len(path) - 1):
        prev = smoothed[-1]
        current = path[i]
        next_point = path[i + 1]
        
        # Check if current point is on the same line as prev and next
        # If not, we need to keep it (it's a turn)
        if not is_on_same_line(prev, current, next_point):
            smoothed.append(current)
    
    smoothed.append(path[-1])  # Always keep goal
    
    return smoothed


def is_on_same_line(p1, p2, p3):
    """
    Check if three points are on the same line (collinear).
    Uses cross product - if it's zero, points are on same line.
    """
    # Vector from p1 to p2
    v1_row = p2[0] - p1[0]
    v1_col = p2[1] - p1[1]
    
    # Vector from p1 to p3
    v2_row = p3[0] - p1[0]
    v2_col = p3[1] - p1[1]
    
    # Cross product (in 2D, this gives us a single number)
    cross = v1_row * v2_col - v1_col * v2_row
    
    # If cross product is 0, points are on same line
    return abs(cross) < 0.001


# ============================================
# EXAMPLE USAGE (for testing)
# ============================================

if __name__ == "__main__":
    # Create a simple grid
    # 0 = free space, 1 = obstacle
    test_grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],  # Wall obstacle
        [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
    
    start = (0, 0)  # Top-left
    goal = (5, 5)   # Somewhere past the wall
    
    print("Finding path...")
    path = find_path(test_grid, start, goal)
    
    if path:
        print(f"Path found with {len(path)} steps:")
        for step, point in enumerate(path):
            print(f"  Step {step}: ({point[0]}, {point[1]})")
        
        # Smooth the path
        smooth = smooth_path(path)
        print(f"\nSmoothed path with {len(smooth)} waypoints:")
        for step, point in enumerate(smooth):
            print(f"  Waypoint {step}: ({point[0]}, {point[1]})")
    else:
        print("No path found!")
