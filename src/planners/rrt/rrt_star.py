import time
from math import sqrt
from random import randint, random
from typing import Callable, List, Optional, Tuple

from src.grid_utils import line_is_clear

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _cost(parent: Optional[Point], costs: dict[Point, float], node: Point) -> float:
    if parent is None:
        return 0.0
    return costs[parent] + _distance(parent, node)


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


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_iter: int = 1200,
    step_size: int = 1,
    goal_sample_rate: float = 0.1,
    goal_tolerance: float = 1.5,
    rewire_radius: float = 4.0,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    tree: dict[Point, Optional[Point]] = {start: None}
    costs: dict[Point, float] = {start: 0.0}
    nodes: List[Point] = [start]
    rrt_lines: List[Tuple[Point, Point]] = []
    grid_size = len(grid)

    for iteration in range(1, max_iter + 1):
        rand_point = goal if random() < goal_sample_rate else (
            randint(0, grid_size - 1),
            randint(0, grid_size - 1),
        )

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
        if grid[step[1]][step[0]] != 0:
            continue
        if not line_is_clear(grid, nearest, step):
            continue

        # choose best parent within radius
        near_nodes = [n for n in nodes if _distance(n, step) <= rewire_radius]
        parent = nearest
        min_cost = costs[nearest] + _distance(nearest, step)
        for n in near_nodes:
            if not line_is_clear(grid, n, step):
                continue
            c = costs[n] + _distance(n, step)
            if c < min_cost:
                min_cost = c
                parent = n
        tree[step] = parent
        costs[step] = min_cost
        nodes.append(step)
        rrt_lines.append((parent, step))  # type: ignore[arg-type]

        # rewire neighbors to this node if cheaper
        for n in near_nodes:
            if n == parent:
                continue
            if not line_is_clear(grid, step, n):
                continue
            new_cost = costs[step] + _distance(step, n)
            if new_cost + 1e-6 < costs.get(n, float("inf")):
                old_parent = tree[n]
                tree[n] = step
                costs[n] = new_cost
                rrt_lines.append((step, n))
                if old_parent:
                    try:
                        rrt_lines.remove((old_parent, n))
                    except ValueError:
                        pass

        if animate and visualize:
            visualize(rrt_lines)

        if _distance(step, goal) <= goal_tolerance and line_is_clear(grid, step, goal):
            tree[goal] = step
            costs[goal] = costs[step] + _distance(step, goal)
            nodes.append(goal)
            current: Optional[Point] = goal
            path: List[Point] = []
            while current:
                path.append(current)
                current = tree[current]
            path.reverse()

            # Densify to ensure continuity
            dense_path: List[Point] = []
            for i in range(len(path) - 1):
                seg = _interpolate(path[i], path[i + 1])
                if dense_path:
                    dense_path.extend(seg[1:])  # avoid duplicates
                else:
                    dense_path.extend(seg)

            return dense_path, int((time.time() - start_time) * 1000), iteration

    return [], int((time.time() - start_time) * 1000), max_iter
