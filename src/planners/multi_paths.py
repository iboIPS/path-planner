import time
from typing import List, Tuple

from src.planners.a_star import run as a_star_run

Point = Tuple[int, int]
Grid = List[List[int]]


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    paths_count: int = 3,
    block_fraction: float = 0.01,  # block ~1% of each found path to allow 99% overlap
    animate: bool = False,
    visualize=None,
) -> tuple[List[dict], int]:
    # We reuse A* repeatedly and block only a tiny portion of each path
    # so subsequent solutions can overlap heavily (up to ~99%).
    base_grid = [row[:] for row in grid]
    working_grid = [row[:] for row in grid]
    results: List[dict] = []
    total_time_ms = 0

    for _ in range(max(1, paths_count)):
        path, duration_ms = a_star_run(working_grid, start, goal, animate=False, visualize=None)
        total_time_ms += duration_ms
        if not path:
            # If blocking made it impossible, fall back to the first path so we still return N entries.
            if results:
                first = results[0]
                while len(results) < paths_count:
                    results.append(
                        {"path": list(first["path"]), "time_ms": first["time_ms"], "length": first["length"]}
                    )
            break

        results.append(
            {
                "path": path,
                "time_ms": duration_ms,
                "length": len(path),
            }
        )

        # Block a sparse subset of the path (except start/goal) to encourage alternate routes.
        interior = path[1:-1]
        if interior:
            block_interval = max(1, int(1 / max(block_fraction, 1e-3)))
            for idx, (x, y) in enumerate(interior):
                if idx % block_interval == 0:
                    working_grid[y][x] = 1

        if animate and visualize:
            visualize(path)

    return results, total_time_ms
