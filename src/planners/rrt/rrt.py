import time
from math import sqrt
from random import randint, random
from typing import Callable, List, Optional, Tuple

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_iter: int = 1000,
    step_size: int = 1,
    goal_sample_rate: float = 0.1,
    goal_tolerance: float = 1.5,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    tree: dict[Point, Optional[Point]] = {start: None}
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

        if grid[step[1]][step[0]] == 0:
            tree[step] = nearest
            nodes.append(step)
            rrt_lines.append((nearest, step))

            if animate and visualize:
                visualize(rrt_lines)

            if _distance(step, goal) <= goal_tolerance:
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
