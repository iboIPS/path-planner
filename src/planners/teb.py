import time
from typing import List, Tuple

from src.grid_utils import line_is_clear
from src.planners.a_star import run as run_a_star

Point = Tuple[int, int]
Grid = List[List[int]]


def _shortcut_path(grid: Grid, path: List[Point]) -> List[Point]:
    if not path:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_is_clear(grid, path[i], path[j]):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    animate: bool = False,
    visualize=None,
) -> tuple[List[Point], int]:
    base_path, base_time = run_a_star(grid, start, goal, animate=False, visualize=None)
    if not base_path:
        return [], base_time
    smoothed = _shortcut_path(grid, base_path)
    if animate and visualize:
        visualize(smoothed)
    return smoothed, base_time
