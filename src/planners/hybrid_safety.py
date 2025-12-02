import heapq
import math
import time
from collections import deque
from typing import Callable, Iterable, List, Optional, Tuple

from src.grid_utils import get_neighbors, in_bounds, manhattan

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


def compute_distance_transform(grid: Grid) -> List[List[float]]:
    """Return a map of Manhattan distance to the nearest obstacle for each cell."""
    size = len(grid)
    dist = [[math.inf for _ in range(size)] for _ in range(size)]
    q: deque[Point] = deque()

    for y, row in enumerate(grid):
        for x, val in enumerate(row):
            if val == 1:
                dist[y][x] = 0.0
                q.append((x, y))

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while q:
        x, y = q.popleft()
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if in_bounds(grid, nx, ny) and grid[ny][nx] == 0:
                nd = dist[y][x] + 1
                if nd < dist[ny][nx]:
                    dist[ny][nx] = nd
                    q.append((nx, ny))

    # Replace unreachable infinities (no obstacles on map) with a large distance.
    fallback = float(size * 2)
    for y in range(size):
        for x in range(size):
            if math.isinf(dist[y][x]):
                dist[y][x] = fallback
    return dist


def safety_cost_at(pos: Point, distance_map: List[List[float]], *, d_min: float, c_max: float) -> float:
    d = distance_map[pos[1]][pos[0]]
    if d <= d_min:
        return c_max
    return 1.0 / d


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    animate: bool = False,
    visualize: Visualizer = None,
    w_length: float = 1.0,
    w_safety: float = 2.0,
    l_ref: float = 1.0,
    d_min: float = 1.5,
    c_max: float = 1000.0,
    goal_tolerance: int = 0,
) -> tuple[List[Point], int]:
    start_time = time.time()

    if grid[start[1]][start[0]] == 1 or grid[goal[1]][goal[0]] == 1:
        return [], int((time.time() - start_time) * 1000)

    distance_map = compute_distance_transform(grid)

    open_set: list[tuple[float, int, Point]] = []
    counter = 0
    h0 = math.hypot(start[0] - goal[0], start[1] - goal[1])
    heapq.heappush(open_set, (h0, counter, start))

    came_from: dict[Point, Point] = {}
    g_score: dict[Point, float] = {start: 0.0}
    visited: set[Point] = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if manhattan(current, goal) <= goal_tolerance:
            path = _reconstruct_path(came_from, current)
            duration_ms = int((time.time() - start_time) * 1000)
            return path, duration_ms

        if current in visited:
            continue
        visited.add(current)

        for neighbor in get_neighbors(grid, current):
            step_length = math.hypot(neighbor[0] - current[0], neighbor[1] - current[1])
            length_cost = step_length / max(l_ref, 1e-6)
            safety_cost = safety_cost_at(neighbor, distance_map, d_min=d_min, c_max=c_max)
            hybrid_step_cost = w_length * length_cost + w_safety * safety_cost

            tentative_g = g_score[current] + hybrid_step_cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                h = math.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1])
                f = tentative_g + h
                counter += 1
                heapq.heappush(open_set, (f, counter, neighbor))
                came_from[neighbor] = current

        if animate and visualize:
            visualize(visited)

    duration_ms = int((time.time() - start_time) * 1000)
    return [], duration_ms
