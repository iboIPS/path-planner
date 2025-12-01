import time
from math import sqrt, cos, sin, atan2
from random import random, uniform
from typing import Callable, List, Optional, Tuple

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _line_collision(grid: Grid, p1: Point, p2: Point) -> bool:
    """Check if line between two points collides with obstacles"""
    x0, y0 = p1
    x1, y1 = p2
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
            return True
        if grid[y][x] == 1:
            return True
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return False


def _interpolate(a: Point, b: Point) -> List[Point]:
    """Bresenham-style interpolation to ensure continuous grid path."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    pts = [(x, y)]
    if dx > dy:
        err = dx // 2
        while x != x1:
            x += sx
            err -= dy
            if err < 0:
                y += sy
                err += dx
            pts.append((x, y))
    else:
        err = dy // 2
        while y != y1:
            y += sy
            err -= dx
            if err < 0:
                x += sx
                err += dy
            pts.append((x, y))
    return pts


def _sample_ellipse(start: Point, goal: Point, c_best: float, c_min: float, grid_size: int) -> Point:
    """Sample point from ellipse focused on start and goal"""
    if c_best == float('inf'):
        return (int(random() * grid_size), int(random() * grid_size))
    
    c_x = (start[0] + goal[0]) / 2
    c_y = (start[1] + goal[1]) / 2
    
    theta = atan2(goal[1] - start[1], goal[0] - start[0])
    
    a = c_best / 2
    b = sqrt(c_best**2 - c_min**2) / 2
    
    # Sample from unit ball
    r = sqrt(random())
    angle = uniform(0, 2 * 3.14159)
    x = r * cos(angle)
    y = r * sin(angle)
    
    # Transform to ellipse
    x_ellipse = a * x
    y_ellipse = b * y
    
    # Rotate and translate
    x_world = x_ellipse * cos(theta) - y_ellipse * sin(theta) + c_x
    y_world = x_ellipse * sin(theta) + y_ellipse * cos(theta) + c_y
    
    return (int(round(x_world)), int(round(y_world)))


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_iter: int = 1000,
    step_size: int = 1,
    goal_sample_rate: float = 0.1,
    goal_tolerance: float = 1.5,
    rewire_radius: float = 5.0,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    """Informed RRT*: uses ellipsoidal sampling after finding initial solution"""
    start_time = time.time()
    tree: dict[Point, Optional[Point]] = {start: None}
    cost: dict[Point, float] = {start: 0.0}
    nodes: List[Point] = [start]
    rrt_lines: List[Tuple[Point, Point]] = []
    grid_size = len(grid)
    
    c_best = float('inf')
    c_min = _distance(start, goal)
    solution_found = False

    for iteration in range(1, max_iter + 1):
        # Use ellipsoidal sampling if solution found
        if solution_found:
            rand_point = _sample_ellipse(start, goal, c_best, c_min, grid_size)
        elif random() < goal_sample_rate:
            rand_point = goal
        else:
            rand_point = (int(random() * grid_size), int(random() * grid_size))

        if not (0 <= rand_point[0] < grid_size and 0 <= rand_point[1] < grid_size):
            continue

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
            nearby = [n for n in nodes if _distance(n, step) <= rewire_radius]
            
            min_cost = float('inf')
            min_parent = nearest
            for node in nearby:
                new_cost = cost[node] + _distance(node, step)
                if new_cost < min_cost and not _line_collision(grid, node, step):
                    min_cost = new_cost
                    min_parent = node
            
            tree[step] = min_parent
            cost[step] = min_cost
            nodes.append(step)
            rrt_lines.append((min_parent, step))
            
            for node in nearby:
                new_cost = cost[step] + _distance(step, node)
                if new_cost < cost[node] and not _line_collision(grid, step, node):
                    old_parent = tree[node]
                    if old_parent:
                        try:
                            rrt_lines.remove((old_parent, node))
                        except ValueError:
                            pass
                    tree[node] = step
                    cost[node] = new_cost
                    rrt_lines.append((step, node))

            if animate and visualize:
                visualize(rrt_lines)

            if _distance(step, goal) <= goal_tolerance:
                goal_cost = cost[step] + _distance(step, goal)
                if goal_cost < c_best:
                    tree[goal] = step
                    cost[goal] = goal_cost
                    if goal not in nodes:
                        nodes.append(goal)
                    c_best = goal_cost
                    solution_found = True

    if solution_found:
        current = goal
        path: List[Point] = []
        while current:
            path.append(current)
            current = tree[current]
        path.reverse()

        dense_path: List[Point] = []
        for i in range(len(path) - 1):
            seg = _interpolate(path[i], path[i + 1])
            if dense_path:
                dense_path.extend(seg[1:])
            else:
                dense_path.extend(seg)

        duration_ms = int((time.time() - start_time) * 1000)
        return dense_path, duration_ms, iteration

    return [], int((time.time() - start_time) * 1000), max_iter
