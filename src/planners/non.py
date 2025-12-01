import heapq
import random
import time
from typing import Dict, List, Tuple

from src.grid_utils import line_is_clear, manhattan

Point = Tuple[int, int]
Grid = List[List[int]]
Graph = Dict[Point, List[Point]]


def _build_graph(grid: Grid, samples: List[Point], k: int = 8) -> Graph:
    graph: Graph = {p: [] for p in samples}
    for i, p in enumerate(samples):
        # find nearest k neighbors (by Manhattan) to reduce dense connections
        neighbors = sorted(
            (s for s in samples if s != p),
            key=lambda s: manhattan(p, s),
        )[:k]
        for n in neighbors:
            if line_is_clear(grid, p, n):
                graph[p].append(n)
                graph[n].append(p)
    return graph


def _shortest_path(graph: Graph, start: Point, goal: Point) -> List[Point]:
    open_set: list[tuple[int, Point]] = [(0, start)]
    came_from: Dict[Point, Point] = {}
    g_score: Dict[Point, int] = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path: List[Point] = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        for neighbor in graph.get(current, []):
            tentative = g_score[current] + manhattan(current, neighbor)
            if tentative < g_score.get(neighbor, 1e9):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(open_set, (tentative, neighbor))
    return []


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    sample_count: int = 200,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    start_time = time.time()
    free_cells = [
        (x, y)
        for y in range(len(grid))
        for x in range(len(grid))
        if grid[y][x] == 0
    ]
    if len(free_cells) < 2:
        return [], int((time.time() - start_time) * 1000)
    samples = [start, goal] + random.sample(free_cells, min(sample_count, len(free_cells)))
    graph = _build_graph(grid, samples)

    path = _shortest_path(graph, start, goal)
    if animate and visualize:
        visualize([])  # no custom animation; reuse visited overlay if provided
    return path, int((time.time() - start_time) * 1000)
