import math
import time
from typing import List, Tuple

from src.grid_utils import neighbors8

Point = Tuple[int, int]
Grid = List[List[int]]


def _angle(from_pt: Point, to_pt: Point) -> float:
    dx = to_pt[0] - from_pt[0]
    dy = to_pt[1] - from_pt[1]
    return math.atan2(dy, dx)


def _histogram(grid: Grid, center: Point, radius: int = 4) -> List[tuple[float, int]]:
    cx, cy = center
    buckets: List[tuple[float, int]] = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            x, y = cx + dx, cy + dy
            if 0 <= x < len(grid) and 0 <= y < len(grid):
                if grid[y][x] == 1:
                    angle = math.atan2(dy, dx)
                    buckets.append((angle, abs(dx) + abs(dy)))
    return buckets


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    path: List[Point] = [start]
    current = start
    visited = set([start])

    for _ in range(300):
        if current == goal:
            break
        target_angle = _angle(current, goal)
        hist = _histogram(grid, current)
        best_next = None
        best_score = math.inf
        for nxt in neighbors8(grid, current):
            if nxt in visited:
                continue
            ang = _angle(current, nxt)
            heading = abs(ang - target_angle)
            obstacle_cost = sum(
                1 / max(1, dist) for a, dist in hist if abs(a - ang) < math.radians(30)
            )
            score = heading + obstacle_cost
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
