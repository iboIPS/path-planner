import heapq
import time
from typing import Iterable, List, Tuple

from src.grid_utils import get_neighbors

Point = Tuple[int, int]
Grid = List[List[int]]


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    open_set: list[tuple[int, Point]] = []
    heapq.heappush(open_set, (0, start))
    came_from: dict[Point, Point] = {}
    g_score: dict[Point, int] = {start: 0}
    visited: set[Point] = set()
    start_time = time.time()

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct
            path: List[Point] = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path, int((time.time() - start_time) * 1000)

        visited.add(current)
        for neighbor in get_neighbors(grid, current):
            tentative = g_score[current] + 1
            if neighbor not in g_score or tentative < g_score[neighbor]:
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative, neighbor))
        if animate and visualize:
            visualize(visited)

    return [], int((time.time() - start_time) * 1000)
