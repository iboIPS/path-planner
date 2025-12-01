import random

CONFIG = {
    "grid_size": 100,   # total map size in cm
    "cell_size": 5      # each cell size in cm
}

class MapGenerator:
    def __init__(self, config: dict[str, int] = None):
        self.config = config if config is not None else CONFIG
        
        self.grid_size = self.config["grid_size"]
        self.cell_size = self.config["cell_size"]
        self.num_cells = self.grid_size // self.cell_size

        # Initialize with all unknown (-1)
        self.grid = [[-1 for _ in range(self.num_cells)] 
                          for _ in range(self.num_cells)]

        # Robot starts at the center
        self.robot_pos = (self.num_cells // 2, self.num_cells // 2)

    def set_cell(self, x: int, y: int, value: int):
        if 0 <= x < self.num_cells and 0 <= y < self.num_cells:
            self.grid[y][x] = value
        else:
            raise IndexError("Cell index out of bounds")

    def display(self):
        for row in self.grid:
            print(" ".join(f"{cell:2}" for cell in row))

    # ─── MAP GENERATION MODES ───────────────────────────────
    def generate_just_started(self, radius: int = 2):
        """
        All cells unknown except around robot's start position.
        radius: how many cells around robot to reveal.
        """
        self.grid = [[-1 for _ in range(self.num_cells)] 
                          for _ in range(self.num_cells)]

        cx, cy = self.robot_pos
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < self.num_cells and 0 <= y < self.num_cells:
                    # immediate surroundings are free (0)
                    self.grid[y][x] = 0  

        # robot's exact position can be marked separately if needed
        self.grid[cy][cx] = 8  # special marker for robot

    def generate_mid_discovery(self, discovery_ratio: float = 0.3):
        """Some random cells are discovered, simulating partial exploration."""
        self.generate_just_started(radius=2)
        total_cells = self.num_cells * self.num_cells
        discovered_cells = int(total_cells * discovery_ratio)

        for _ in range(discovered_cells):
            x = random.randint(0, self.num_cells - 1)
            y = random.randint(0, self.num_cells - 1)
            if self.grid[y][x] == -1:  # only update unknown cells
                self.grid[y][x] = random.choice([0, 1])  # free or occupied

    def generate_fully_discovered(self):
        """All cells are either free or occupied."""
        self.grid = [[random.choice([0, 1]) for _ in range(self.num_cells)] 
                          for _ in range(self.num_cells)]


# ─── TEST ─────────────────────────────────────────────
if __name__ == "__main__":
    mg = MapGenerator()

    print("\n=== Just Started (knows surroundings) ===")
    mg.generate_just_started(radius=2)
    mg.display()

    print("\n=== Mid Discovery ===")
    mg.generate_mid_discovery(discovery_ratio=0.5)
    mg.display()

    print("\n=== Fully Discovered ===")
    mg.generate_fully_discovered()
    mg.display()
