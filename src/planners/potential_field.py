import math
import time
from typing import List, Tuple

from src.grid_utils import neighbors8

Point = Tuple[int, int]
Grid = List[List[int]]


def _repulsive(grid: Grid, point: Point, radius: int = 4, weight: float = 20.0) -> float:
    x0, y0 = point
    force = 0.0
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if 0 <= x < len(grid) and 0 <= y < len(grid) and grid[y][x] == 1:
                dist = math.hypot(dx, dy)
                if dist > 0:
                    force += weight / (dist**2)
    return force


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    max_steps: int = 400,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    path: List[Point] = [start]
    current = start
    visited = set([start])

    for _ in range(max_steps):
        if current == goal:
            break
        best_next = None
        best_energy = math.inf
        for nxt in neighbors8(grid, current):
            if nxt in visited:
                continue
            attract = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            repel = _repulsive(grid, nxt)
            energy = attract + repel
            if energy < best_energy:
                best_energy = energy
                best_next = nxt
        if not best_next:
            break
        path.append(best_next)
        current = best_next
        visited.add(current)
        if animate and visualize:
            visualize(path)

    return path if path[-1] == goal else [], int((time.time() - start_time) * 1000)
