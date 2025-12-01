import time
from math import sqrt
from random import randint, random
from typing import Callable, List, Optional, Tuple

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _extend_tree(
    tree: dict[Point, Optional[Point]],
    nodes: List[Point],
    target: Point,
    grid: Grid,
    step_size: int,
    rrt_lines: List[Tuple[Point, Point]]
) -> Optional[Point]:
    """Extend tree toward target point"""
    grid_size = len(grid)
    nearest = min(nodes, key=lambda n: _distance(n, target))
    
    dir_x = target[0] - nearest[0]
    dir_y = target[1] - nearest[1]
    length = sqrt(dir_x**2 + dir_y**2)
    if length == 0:
        return None
    
    step = (
        int(round(nearest[0] + step_size * dir_x / length)),
        int(round(nearest[1] + step_size * dir_y / length)),
    )
    
    if not (0 <= step[0] < grid_size and 0 <= step[1] < grid_size):
        return None
    
    if grid[step[1]][step[0]] == 0:
        tree[step] = nearest
        nodes.append(step)
        rrt_lines.append((nearest, step))
        return step
    
    return None


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_iter: int = 1000,
    step_size: int = 1,
    goal_tolerance: float = 1.5,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    """RRT-Connect: bidirectional search from start and goal"""
    start_time = time.time()
    
    # Start tree
    tree_start: dict[Point, Optional[Point]] = {start: None}
    nodes_start: List[Point] = [start]
    
    # Goal tree
    tree_goal: dict[Point, Optional[Point]] = {goal: None}
    nodes_goal: List[Point] = [goal]
    
    rrt_lines: List[Tuple[Point, Point]] = []
    grid_size = len(grid)

    for i in range(max_iter):
        # Random point
        rand_point = (randint(0, grid_size - 1), randint(0, grid_size - 1))
        
        # Extend start tree toward random point
        new_node = _extend_tree(tree_start, nodes_start, rand_point, grid, step_size, rrt_lines)
        
        if new_node and animate and visualize:
            visualize(rrt_lines)
        
        if new_node:
            # Try to connect goal tree to new node
            for _ in range(5):  # Multiple connection attempts
                connect_node = _extend_tree(tree_goal, nodes_goal, new_node, grid, step_size, rrt_lines)
                
                if connect_node and animate and visualize:
                    visualize(rrt_lines)
                
                if connect_node and _distance(connect_node, new_node) <= goal_tolerance:
                    # Trees connected! Build path
                    path_start: List[Point] = []
                    current = new_node
                    while current:
                        path_start.append(current)
                        current = tree_start[current]
                    path_start.reverse()
                    
                    path_goal: List[Point] = []
                    current = connect_node
                    while current:
                        path_goal.append(current)
                        current = tree_goal[current]
                    
                    path = path_start + path_goal
                    duration_ms = int((time.time() - start_time) * 1000)
                    return path, duration_ms
                
                if not connect_node:
                    break
        
        # Swap trees for balanced growth
        tree_start, tree_goal = tree_goal, tree_start
        nodes_start, nodes_goal = nodes_goal, nodes_start

    return [], int((time.time() - start_time) * 1000)