import math
import time
from typing import List, Tuple

from src.grid_utils import neighbors8

Point = Tuple[int, int]
Grid = List[List[int]]


def _clearance(grid: Grid, point: Point, radius: int = 3) -> int:
    x0, y0 = point
    best = radius + 1
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if 0 <= x < len(grid) and 0 <= y < len(grid):
                if grid[y][x] == 1:
                    best = min(best, abs(dx) + abs(dy))
    return best if best <= radius else radius


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    horizon: int = 200,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    path: List[Point] = [start]
    current = start
    visited = set([start])

    for _ in range(horizon):
        if current == goal:
            break
        best_score = math.inf
        best_next = None
        for nxt in neighbors8(grid, current):
            if nxt in visited:
                continue
            heading = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
            clearance = _clearance(grid, nxt)
            velocity_cost = 1  # single step
            score = heading - 0.5 * clearance + 0.1 * velocity_cost
            if score < best_score:
                best_score = score
                best_next = nxt
        if not best_next:
            break
        path.append(best_next)
        current = best_next
        visited.add(current)
        if animate and visualize:
            visualize(path)

    return path if path[-1] == goal else [], int((time.time() - start_time) * 1000)
