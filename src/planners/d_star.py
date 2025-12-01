from typing import List, Tuple

from .a_star import run as a_star_run

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
    # Simple placeholder that reuses A* in reverse.
    path, duration_ms = a_star_run(grid, goal, start, animate=animate, visualize=visualize)
    path.reverse()
    return path, duration_ms
