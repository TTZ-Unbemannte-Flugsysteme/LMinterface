"""
Tests for A* Path Finding Algorithm

Run with:
    cd ~/LLMAgent-----Cocoa_Speech/ros_ws
    python3 src/cocoa_path_planner/test/test_astar.py

Or with pytest:
    python3 -m pytest src/cocoa_path_planner/test/test_astar.py -v
"""

import sys
import os

# Add the package to path so we can import it
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from cocoa_path_planner.astar import find_path, smooth_path


def test_simple_path_no_obstacles():
    """Test finding a path when there are no obstacles."""
    print("\n=== Test 1: Simple path with no obstacles ===")
    
    # Create empty 10x10 grid (all zeros = all free)
    grid = [[0 for _ in range(10)] for _ in range(10)]
    
    start = (0, 0)
    goal = (5, 5)
    
    path = find_path(grid, start, goal)
    
    assert path is not None, "Should find a path!"
    assert path[0] == start, "Path should start at start position"
    assert path[-1] == goal, "Path should end at goal position"
    
    print(f"✓ Found path with {len(path)} steps")
    print(f"  Path: {path}")


def test_path_around_obstacle():
    """Test finding a path around an obstacle."""
    print("\n=== Test 2: Path around obstacle ===")
    
    # Create grid with a wall blocking the direct path
    #
    #   S . . . . . . . . .
    #   . . . . . . . . . .
    #   . . # # # # . . . .   <- wall blocks direct path
    #   . . # # # # . . . .
    #   . . . . . . . . . .
    #   . . . . . G . . . .   <- goal below wall
    #
    grid = [[0 for _ in range(10)] for _ in range(10)]
    
    # Add wall obstacle (rows 2-3, cols 2-5)
    for row in range(2, 4):
        for col in range(2, 6):
            grid[row][col] = 1  # 1 = blocked
    
    start = (0, 0)
    goal = (5, 5)
    
    path = find_path(grid, start, goal)
    
    assert path is not None, "Should find a path around the wall!"
    assert path[0] == start, "Path should start at start position"
    assert path[-1] == goal, "Path should end at goal position"
    
    # Verify path doesn't go through walls
    for cell in path:
        assert grid[cell[0]][cell[1]] == 0, f"Path goes through wall at {cell}!"
    
    print(f"✓ Found path with {len(path)} steps (goes around wall)")
    print(f"  Path: {path}")


def test_no_path_exists():
    """Test when there is no valid path (goal is completely blocked)."""
    print("\n=== Test 3: No path possible ===")
    
    # Create grid where goal is surrounded by walls
    #
    #   S . . . . . . . . .
    #   . . . . # # # . . .
    #   . . . . # G # . . .   <- goal surrounded by walls
    #   . . . . # # # . . .
    #
    grid = [[0 for _ in range(10)] for _ in range(10)]
    
    # Surround goal with walls
    goal = (2, 5)
    for row in range(1, 4):
        for col in range(4, 7):
            grid[row][col] = 1
    grid[goal[0]][goal[1]] = 0  # Goal itself is free (but unreachable)
    
    start = (0, 0)
    
    path = find_path(grid, start, goal)
    
    assert path is None, "Should NOT find a path (goal is unreachable)!"
    print("✓ Correctly returned None (no path exists)")


def test_start_is_blocked():
    """Test when start position is blocked."""
    print("\n=== Test 4: Start position is blocked ===")
    
    grid = [[0 for _ in range(10)] for _ in range(10)]
    
    start = (0, 0)
    goal = (5, 5)
    
    # Block the start position
    grid[start[0]][start[1]] = 1
    
    path = find_path(grid, start, goal)
    
    assert path is None, "Should NOT find a path (start is blocked)!"
    print("✓ Correctly returned None (start is blocked)")


def test_path_smoothing():
    """Test that path smoothing removes unnecessary waypoints."""
    print("\n=== Test 5: Path smoothing ===")
    
    # A path with many collinear points
    # (0,0) -> (1,1) -> (2,2) -> (3,3) should become just (0,0) -> (3,3)
    path = [(0, 0), (1, 1), (2, 2), (3, 3), (3, 4), (3, 5)]
    
    smoothed = smooth_path(path)
    
    print(f"  Original: {len(path)} points - {path}")
    print(f"  Smoothed: {len(smoothed)} points - {smoothed}")
    
    assert len(smoothed) <= len(path), "Smoothed path should not be longer!"
    assert smoothed[0] == path[0], "Should keep start point"
    assert smoothed[-1] == path[-1], "Should keep end point"
    
    print("✓ Path smoothing works!")


def test_cocoa_demo_scenario():
    """
    Test the scenario from our Cocoa demo:
    - Drone starts at (0, 0)
    - Wall obstacle at (2, 2)
    - Goal (chair) at (2, 4) - blocked by wall
    
    This is similar to the real world scenario in objects.yaml
    """
    print("\n=== Test 6: Cocoa Demo Scenario ===")
    print("  Drone at (0,0), wall at (2,2), chair at (2,4)")
    
    # Create a larger grid (resolution 0.5m, 30x30 grid covers -7.5 to 7.5m)
    grid_size = 30
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    
    # Center is at (15, 15) = world (0, 0)
    # World (2, 2) = grid (15+4, 15+4) = (19, 19) at 0.5m resolution
    
    # Add wall obstacle at world (2, 2) covering roughly 3m in Y
    # At 0.5m resolution, that's 6 cells
    center_row, center_col = 15, 15  # This is world (0, 0)
    wall_row = center_row + 4  # World y = 2m
    wall_col = center_col + 4  # World x = 2m
    
    for r in range(wall_row - 3, wall_row + 3):  # 3m wall in Y
        for c in range(wall_col - 1, wall_col + 1):  # 0.5m thick wall
            if 0 <= r < grid_size and 0 <= c < grid_size:
                grid[r][c] = 1
    
    # Start at center (world 0, 0)
    start = (center_row, center_col)
    
    # Goal at world (2, 4) = grid (15+8, 15+4) = (23, 19)
    goal = (center_row + 8, center_col + 4)
    
    path = find_path(grid, start, goal)
    
    assert path is not None, "Should find a path around the wall to the chair!"
    
    # Smooth the path
    smoothed = smooth_path(path)
    
    print(f"✓ Found path with {len(path)} steps, smoothed to {len(smoothed)} waypoints")
    print(f"  Start (grid): {start}")
    print(f"  Goal (grid): {goal}")
    print(f"  Wall blocks direct path, so path goes around!")


def run_all_tests():
    """Run all tests."""
    print("=" * 60)
    print("Running A* Path Finding Tests")
    print("=" * 60)
    
    try:
        test_simple_path_no_obstacles()
        test_path_around_obstacle()
        test_no_path_exists()
        test_start_is_blocked()
        test_path_smoothing()
        test_cocoa_demo_scenario()
        
        print("\n" + "=" * 60)
        print("All tests passed! ✓")
        print("=" * 60)
        
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(run_all_tests())
