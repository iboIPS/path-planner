# Path Planner GUI

An interactive sandbox for comparing grid-based path planning algorithms. The app uses Pygame for rendering and provides a simple UI to draw obstacles, set start/goal locations, and watch planners animate their search.

## 🚀 Clone This Repository

### Standard clone
```bash
git clone https://github.com/iboIPS/path-planner.git
cd path-planner
git submodule update --init --recursive
```

## Features
- Multiple planners: A*, Dijkstra, D*, RRT variants (classic, *, Connect, Dynamic, Informed, Goal-Biased), PRM variants (standard, *, Lazy, Gaussian, Bridge), DWA, TEB, VFH, Potential Field, and a MultiPaths sampler.
- Interactive map editing: paint/erase obstacles, set start/goal, undo/redo, random map generation, and random start/goal placement.
- Animation modes: step through explored cells or RRT/PRM trees, adjust animation delay, and toggle a legend for multiple sampled paths.
- Map import: load a PNG/JPG map (dark = obstacle, light = free) and resize to the working grid.
- Tunable parameters: adjust RRT iterations and PRM samples directly from the UI.

## Requirements
- Python 3.11+
- Runtime deps: `pygame`, `numpy`, `pillow`, and `tk` (for the file picker)
- Optional: [pixi](https://pixi.sh/) if you prefer the included Conda-based workflow (`pixi.lock`)

## Setup
Choose either pip/venv or pixi:

**pip/venv**
```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
python -m pip install --upgrade pip
python -m pip install pygame==2.6.1 numpy==2.2.6 "pillow>=12,<13"
python app.py
```

**pixi**
```bash
pixi install
pixi run python app.py
```

## Controls (quick reference)
- Mouse: left = draw obstacles, right = erase, middle = set start then goal (alternates)
- Space = run selected planner; click planner buttons to switch modes
- p = select solution
- A = toggle animation; `[` / `]` = slow/fast animation
- M = random map; G = random start/goal; R = reset grid
- Ctrl+U = undo; Y = redo; H = toggle on-screen hints
- Right panel buttons: simulate (forces animate), random goal/map, load map image, RRT/PRM parameter tweaks

## Project layout
- `app.py`: application entry point and main event loop
- `src/config.py`: grid sizing, window dimensions, and colors
- `src/state.py`: UI/state management, history, and map import helpers
- `src/render.py`: drawing routines, UI layout, and hint overlay
- `src/planners/`: implementations for each planner (RRT/PRM subpackages included)

## License
This project is licensed under the MIT License (see `LICENSE`).
