import heapq
import time
from typing import Callable, Iterable, List, Optional, Tuple

from src.grid_utils import get_neighbors, manhattan

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[Iterable[Point]], None]]


def _reconstruct_path(came_from: dict[Point, Point], current: Point) -> List[Point]:
    path: List[Point] = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    animate: bool = False,
    visualize: Visualizer = None,
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
            duration_ms = int((time.time() - start_time) * 1000)
            return _reconstruct_path(came_from, current), duration_ms

        visited.add(current)

        for neighbor in get_neighbors(grid, current):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current

        if animate and visualize:
            visualize(visited)

    duration_ms = int((time.time() - start_time) * 1000)
    return [], duration_ms
