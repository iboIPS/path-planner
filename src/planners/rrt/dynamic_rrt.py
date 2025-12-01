import time
from math import sqrt
from random import randint, random
from typing import Callable, List, Optional, Tuple, Set

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _check_path_validity(grid: Grid, node: Point, parent: Optional[Point]) -> bool:
    """Check if connection between node and parent is still valid"""
    if parent is None:
        return True
    
    x0, y0 = parent
    x1, y1 = node
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if not (0 <= x < len(grid) and 0 <= y < len(grid)):
            return False
        if grid[y][x] == 1:
            return False
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return True


def _prune_invalid_nodes(
    tree: dict[Point, Optional[Point]],
    nodes: List[Point],
    grid: Grid,
    rrt_lines: List[Tuple[Point, Point]]
) -> Set[Point]:
    """Remove nodes that are now in collision or have invalid parent connections"""
    invalid_nodes: Set[Point] = set()
    grid_size = len(grid)
    
    for node in nodes[:]:
        # Check if node itself is in collision
        if not (0 <= node[0] < grid_size and 0 <= node[1] < grid_size):
            invalid_nodes.add(node)
            continue
            
        if grid[node[1]][node[0]] == 1:
            invalid_nodes.add(node)
            continue
        
        # Check if connection to parent is valid
        parent = tree.get(node)
        if not _check_path_validity(grid, node, parent):
            invalid_nodes.add(node)
    
    # Remove invalid nodes and their children
    nodes_to_remove = set(invalid_nodes)
    changed = True
    while changed:
        changed = False
        for node in list(nodes):
            if node in nodes_to_remove:
                continue
            parent = tree.get(node)
            if parent in nodes_to_remove:
                nodes_to_remove.add(node)
                changed = True
    
    # Update tree and lines
    for node in nodes_to_remove:
        if node in tree:
            parent = tree[node]
            if parent:
                line = (parent, node)
                if line in rrt_lines:
                    rrt_lines.remove(line)
            del tree[node]
        if node in nodes:
            nodes.remove(node)
    
    return nodes_to_remove


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_iter: int = 1000,
    step_size: int = 1,
    goal_sample_rate: float = 0.1,
    goal_tolerance: float = 1.5,
    replan_frequency: int = 50,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    """Dynamic RRT: handles changing environments by pruning invalid nodes"""
    start_time = time.time()
    tree: dict[Point, Optional[Point]] = {start: None}
    nodes: List[Point] = [start]
    rrt_lines: List[Tuple[Point, Point]] = []
    grid_size = len(grid)

    for iteration in range(1, max_iter + 1):
        # Periodically check for invalid nodes due to environment changes
        if iteration % replan_frequency == 0 and iteration > 0:
            removed = _prune_invalid_nodes(tree, nodes, grid, rrt_lines)
            if start in removed:
                # Start position became invalid
                return [], int((time.time() - start_time) * 1000)
        
        rand_point = goal if random() < goal_sample_rate else (
            randint(0, grid_size - 1),
            randint(0, grid_size - 1),
        )

        if not nodes:
            return [], int((time.time() - start_time) * 1000)

        nearest = min(nodes, key=lambda n: _distance(n, rand_point))

        dir_x = rand_point[0] - nearest[0]
        dir_y = rand_point[1] - nearest[1]
        length = sqrt(dir_x**2 + dir_y**2)
        if length == 0:
            continue

        step = (
            int(round(nearest[0] + step_size * dir_x / length)),
            int(round(nearest[1] + step_size * dir_y / length)),
        )

        if not (0 <= step[0] < grid_size and 0 <= step[1] < grid_size):
            continue

        if grid[step[1]][step[0]] == 0:
            tree[step] = nearest
            nodes.append(step)
            rrt_lines.append((nearest, step))

            if animate and visualize:
                visualize(rrt_lines)

            if _distance(step, goal) <= goal_tolerance:
                # Verify path is valid before returning
                if _check_path_validity(grid, goal, step):
                    tree[goal] = step
                    nodes.append(goal)
                    current = goal
                    path: List[Point] = []
                    while current:
                        path.append(current)
                        current = tree[current]
                    path.reverse()
                    duration_ms = int((time.time() - start_time) * 1000)
                    return path, duration_ms, iteration

    return [], int((time.time() - start_time) * 1000), max_iter
