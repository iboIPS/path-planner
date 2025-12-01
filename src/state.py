import random
from pathlib import Path

import numpy as np
from PIL import Image
from copy import deepcopy
from typing import List, Optional, Tuple

Grid = List[List[int]]
Point = Tuple[int, int]


class AppState:
    """Holds grid data, history, and planner configuration."""

    def __init__(self, grid_size: int, cell_size: int) -> None:
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.history: List[Grid] = []
        self.future: List[Grid] = []
        self.grid: Grid = self._empty_grid()
        self.start_pos: Optional[Point] = None
        self.goal_pos: Optional[Point] = None
        self.setting_state = "start"
        self.path: List[Point] = []
        self.planner_mode = "A*"
        self.last_path_time_ms = 0
        self.last_path_length = 0
        self.animate = False
        self.animation_delay = 5  # milliseconds per frame
        self.show_hints = False  # toggle in UI
        self.multi_paths: List[dict] = []  # store multiple path results
        self.multi_count: int = 20
        self.planner_status: str = "idle"  # idle | running | done | failed
        self.solution_iter: Optional[int] = None
        self.rrt_max_iter: int = 1000
        self.prm_samples: int = 500

    def _empty_grid(self) -> Grid:
        return [[0 for _ in range(self.grid_size)] for _ in range(self.grid_size)]

    def save_snapshot(self) -> None:
        self.history.append(deepcopy(self.grid))
        if len(self.history) > 50:
            self.history.pop(0)
        self.future.clear()

    def reset_grid(self) -> None:
        self.save_snapshot()
        self.grid = self._empty_grid()
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.setting_state = "start"
        self.multi_paths = []

    def set_cell_at_pixel(self, pos: Tuple[int, int], value: int) -> None:
        x, y = pos[0] // self.cell_size, pos[1] // self.cell_size
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            if (x, y) != self.start_pos and (x, y) != self.goal_pos:
                if self.grid[y][x] != value:
                    self.save_snapshot()
                    self.grid[y][x] = value

    def set_start_or_goal_at_pixel(self, pos: Tuple[int, int]) -> None:
        x, y = pos[0] // self.cell_size, pos[1] // self.cell_size
        if self.grid[y][x] == 1:
            return
        self.save_snapshot()
        if self.setting_state == "start":
            self.start_pos = (x, y)
            self.setting_state = "goal"
        else:
            self.goal_pos = (x, y)
            self.setting_state = "start"

    def generate_random_map(self, fill_ratio: float = 0.2) -> None:
        self.save_snapshot()
        self.path = []
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if (x, y) != self.start_pos and (x, y) != self.goal_pos:
                    self.grid[y][x] = 1 if random.random() < fill_ratio else 0

    def load_map_from_image(self, filename: str, invert: bool = False) -> None:
        """Load a grid map from a PNG/JPG image. Dark=obstacle, light=free."""
        try:
            img_path = Path(filename)
            if not img_path.exists():
                print(f"Image not found: {filename}")
                return
            img = Image.open(img_path).convert("L")
            img = img.resize((self.grid_size, self.grid_size), Image.NEAREST)
            arr = np.array(img)
            grid_arr = (arr < 128).astype(int)
            if invert:
                grid_arr = 1 - grid_arr

            self.save_snapshot()
            self.grid = grid_arr.tolist()

            # Keep start/goal free if set
            if self.start_pos:
                sx, sy = self.start_pos
                self.grid[sy][sx] = 0
            if self.goal_pos:
                gx, gy = self.goal_pos
                self.grid[gy][gx] = 0

            self.path = []
            self.multi_paths = []
            self.planner_status = "idle"
            self.solution_iter = None
            print(f"Loaded map from {img_path}")
        except Exception as exc:
            print(f"Failed to load map: {exc}")

    def generate_random_start_goal(self) -> None:
        free_cells = [
            (x, y)
            for y in range(self.grid_size)
            for x in range(self.grid_size)
            if self.grid[y][x] == 0
        ]
        if len(free_cells) < 2:
            return

        self.save_snapshot()
        self.start_pos, self.goal_pos = random.sample(free_cells, 2)
        self.setting_state = "start"

    def undo(self) -> bool:
        if not self.history:
            return False
        self.future.append(deepcopy(self.grid))
        self.grid = deepcopy(self.history.pop())
        self.path = []
        return True

    def redo(self) -> bool:
        if not self.future:
            return False
        self.history.append(deepcopy(self.grid))
        self.grid = deepcopy(self.future.pop())
        self.path = []
        return True

    def clear_path(self) -> None:
        self.path = []
        self.last_path_time_ms = 0
        self.last_path_length = 0
        self.multi_paths = []
        self.planner_status = "idle"
        self.solution_iter = None

    def update_path(self, path: List[Point], duration_ms: int, iteration: Optional[int] = None) -> None:
        self.path = list(path)
        self.last_path_time_ms = duration_ms
        self.last_path_length = len(path)
        self.multi_paths = []
        self.planner_status = "done" if path else "failed"
        self.solution_iter = iteration

    def update_multi_paths(self, results: List[dict], total_ms: int) -> None:
        self.multi_paths = results
        self.last_path_time_ms = total_ms
        self.last_path_length = sum(p["length"] for p in results) if results else 0
        self.path = []
        self.planner_status = "done" if results else "failed"
        self.solution_iter = None

    @property
    def has_start_and_goal(self) -> bool:
        return self.start_pos is not None and self.goal_pos is not None
