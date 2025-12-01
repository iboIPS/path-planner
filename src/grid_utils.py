from typing import Iterable, Iterator, List, Tuple

Point = Tuple[int, int]
Grid = List[List[int]]


def manhattan(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(grid: Grid, node: Point) -> Iterable[Point]:
    yield from neighbors4(grid, node)


def in_bounds(grid: Grid, x: int, y: int) -> bool:
    grid_size = len(grid)
    return 0 <= x < grid_size and 0 <= y < grid_size


def is_free(grid: Grid, x: int, y: int) -> bool:
    return in_bounds(grid, x, y) and grid[y][x] == 0


def neighbors4(grid: Grid, node: Point) -> Iterator[Point]:
    x, y = node
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if is_free(grid, nx, ny):
            yield (nx, ny)


def neighbors8(grid: Grid, node: Point) -> Iterator[Point]:
    x, y = node
    for dx, dy in [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ]:
        nx, ny = x + dx, y + dy
        if is_free(grid, nx, ny):
            yield (nx, ny)


def line_is_clear(grid: Grid, a: Point, b: Point) -> bool:
    """Bresenham line check for collision-free segment."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    if dy <= dx:
        err = dx / 2.0
        while x != x1:
            if not is_free(grid, x, y):
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if not is_free(grid, x, y):
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return is_free(grid, x1, y1)
