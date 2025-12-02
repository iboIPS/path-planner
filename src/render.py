from typing import Dict, Iterable, List, Optional, Tuple

import pygame

from src.config import (
    BLUE,
    CELL_SIZE,
    COLORS,
    GRAY,
    GREEN,
    HEIGHT,
    PATH_COLORS,
    RED,
    WHITE,
    UI_PANEL_WIDTH,
    WIDTH,
    WINDOW_WIDTH,
    WINDOW_HEIGHT,
)
from src.planners import PLANNERS
from src.state import AppState

Point = Tuple[int, int]
ACTION_BUTTONS = [
    {"id": "simulate", "label": "Simulate (Animate)"},
    {"id": "random_goal", "label": "Random Goal"},
    {"id": "random_map", "label": "Random Map"},
    {"id": "load_image", "label": "Load Map Image"},
    {"id": "choose_planner", "label": "Choose Planner (type)"},
    {"id": "rrt_inc", "label": "RRT Iter +100"},
    {"id": "rrt_dec", "label": "RRT Iter -100"},
    {"id": "prm_inc", "label": "PRM Samples +100"},
    {"id": "prm_dec", "label": "PRM Samples -100"},
    {"id": "hybrid_len_inc", "label": "Hybrid w_len +0.1"},
    {"id": "hybrid_len_dec", "label": "Hybrid w_len -0.1"},
    {"id": "hybrid_safe_inc", "label": "Hybrid w_safe +0.1"},
    {"id": "hybrid_safe_dec", "label": "Hybrid w_safe -0.1"},
    {"id": "hybrid_dmin_inc", "label": "Hybrid d_min +0.5"},
    {"id": "hybrid_dmin_dec", "label": "Hybrid d_min -0.5"},
    {"id": "hybrid_cmax_inc", "label": "Hybrid c_max +100"},
    {"id": "hybrid_cmax_dec", "label": "Hybrid c_max -100"},
    {"id": "hybrid_lref_inc", "label": "Hybrid L_ref +0.5"},
    {"id": "hybrid_lref_dec", "label": "Hybrid L_ref -0.5"},
    {"id": "hybrid_set", "label": "Set Hybrid Params (type)"},
]
HINT_LINES = [
    "Left click: draw obstacles",
    "Right click: erase obstacles",
    "Middle click: set start then goal (alternates)",
    "Space: run selected planner",
    "Press P to open planner list overlay",
    "HybridSafety planner blends path length with obstacle clearance (P toggles list, I edits params)",
    "I: type HybridSafety params | P: open planner overlay",
    "1-3: A*, D*, RRT",
    "4-6: Dijkstra, RRT*, PRM",
    "7-0: DWA, TEB, VFH, Potential Field",
    "U: MultiPaths mode (default N=20) | Up/Down: adjust N",
    "A: toggle animation, [/]: adjust speed",
    "M: random map, G: random start/goal",
    "R: reset grid, Ctrl+U: undo, Y: redo",
    "H: toggle this hint overlay",
]


def init_display() -> tuple[pygame.Surface, pygame.font.Font]:
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Path Planner Tester")
    font = pygame.font.SysFont("Arial", 16)
    return screen, font


def draw_grid(
    screen: pygame.Surface,
    state: AppState,
    *,
    visited: Optional[Iterable[Point]] = None,
    tree_lines: Optional[List[Tuple[Point, Point]]] = None,
) -> None:
    visited_set = set(visited) if visited is not None else set()
    multi_paths = state.multi_paths if state.multi_paths else []

    for y in range(state.grid_size):
        for x in range(state.grid_size):
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)

            if (x, y) == state.start_pos:
                color = GREEN
            elif (x, y) == state.goal_pos:
                color = RED
            elif (x, y) in state.path:
                color = BLUE
            elif visited_set and (x, y) in visited_set:
                color = (180, 180, 255)
            else:
                color = COLORS["black"] if state.grid[y][x] == 1 else WHITE

            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)

    for idx, info in enumerate(multi_paths):
        path_color = PATH_COLORS[idx % len(PATH_COLORS)]
        for (px, py) in info["path"]:
            rect = pygame.Rect(px * CELL_SIZE, py * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, path_color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)

    if tree_lines:
        for parent, child in tree_lines:
            pygame.draw.line(
                screen,
                (100, 100, 255),
                (
                    parent[0] * CELL_SIZE + CELL_SIZE // 2,
                    parent[1] * CELL_SIZE + CELL_SIZE // 2,
                ),
                (
                    child[0] * CELL_SIZE + CELL_SIZE // 2,
                    child[1] * CELL_SIZE + CELL_SIZE // 2,
                ),
                1,
            )


def draw_stats(screen: pygame.Surface, font: pygame.font.Font, state: AppState) -> None:
    status = state.planner_status
    status_text = (
        "Running..."
        if status == "running"
        else ("Done" if status == "done" else ("Failed" if status == "failed" else "Idle"))
    )
    iter_text = f"Iter found: {state.solution_iter}" if state.solution_iter is not None else ""
    if state.multi_paths:
        text = (
            f"Planner: MultiPaths (N={state.multi_count}, found {len(state.multi_paths)}) | "
            f"Total Time: {state.last_path_time_ms} ms   Anim Delay: {state.animation_delay} ms"
        )
    else:
        text = (
            f"Planner: {state.planner_mode} | Time: {state.last_path_time_ms} ms   "
            f"Path Length: {state.last_path_length * 5} cm   Anim Delay: {state.animation_delay} ms"
        )
    status_surface = font.render(f"Status: {status_text}", True, (0, 0, 0))
    iter_surface = font.render(iter_text, True, (0, 0, 0)) if iter_text else None
    stats_surface = font.render(text, True, (0, 0, 0))
    pygame.draw.rect(screen, GRAY, (0, HEIGHT, WINDOW_WIDTH, 30))
    screen.blit(stats_surface, (10, HEIGHT + 5))
    x = WINDOW_WIDTH - 400
    screen.blit(status_surface, (x, HEIGHT + 5))
    if iter_surface:
        screen.blit(iter_surface, (x + status_surface.get_width() + 8, HEIGHT + 5))


def draw_multi_legend(screen: pygame.Surface, font: pygame.font.Font, state: AppState) -> None:
    if not state.multi_paths:
        return
    y = 10
    x = WIDTH - 220
    header = font.render("Paths Legend:", True, (0, 0, 0))
    screen.blit(header, (x, y))
    y += header.get_height() + 4
    for idx, info in enumerate(state.multi_paths):
        color = PATH_COLORS[idx % len(PATH_COLORS)]
        swatch = pygame.Surface((18, 18))
        swatch.fill(color)
        screen.blit(swatch, (x, y))
        label = font.render(
            f"#{idx+1}: {info['length']*5} cm, {info['time_ms']} ms",
            True,
            (0, 0, 0),
        )
        screen.blit(label, (x + 24, y + 2))
        y += swatch.get_height() + 4


def draw_hints(screen: pygame.Surface, font: pygame.font.Font) -> None:
    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
    overlay.fill((255, 255, 255, 230))
    screen.blit(overlay, (0, 0))

    y = 10
    header = font.render("Hints / Controls", True, (0, 0, 0))
    screen.blit(header, (10, y))
    y += header.get_height() + 6

    for line in HINT_LINES:
        surf = font.render(line, True, (20, 20, 20))
        screen.blit(surf, (10, y))
        y += surf.get_height() + 4


def draw_planner_overlay(screen: pygame.Surface, font: pygame.font.Font, state: AppState) -> None:
    if not state.planner_overlay:
        return
    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
    overlay.fill((255, 255, 255, 235))
    screen.blit(overlay, (0, 0))

    x = 30
    y = 30
    header = font.render("Planner Select (P toggles, Up/Down + Enter, Esc closes)", True, (0, 0, 0))
    screen.blit(header, (x, y))
    y += header.get_height() + 10

    for idx, name in enumerate(state.planner_menu or list(PLANNERS.keys())):
        active = idx == state.planner_cursor
        bg_rect = pygame.Rect(x, y, 260, 24)
        pygame.draw.rect(screen, (210, 230, 255) if active else (235, 235, 235), bg_rect, border_radius=4)
        pygame.draw.rect(screen, GRAY, bg_rect, 1, border_radius=4)
        label = font.render(name, True, (0, 0, 0))
        screen.blit(label, (x + 8, y + 4))
        y += bg_rect.height + 4


def draw_hybrid_input_overlay(screen: pygame.Surface, font: pygame.font.Font, state: AppState) -> None:
    if not state.hybrid_input_active:
        return
    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
    overlay.fill((255, 255, 255, 235))
    screen.blit(overlay, (0, 0))

    x = 30
    y = 30
    lines = [
        "HybridSafety Params (I to start, Enter to apply, Esc to cancel)",
        "Format: w_length, w_safety, L_ref, d_min, c_max",
        f"Current: {state.hybrid_input_text}",
    ]
    for line in lines:
        surf = font.render(line, True, (0, 0, 0))
        screen.blit(surf, (x, y))
        y += surf.get_height() + 6


def layout_action_buttons(font: pygame.font.Font) -> List[Dict[str, pygame.Rect]]:
    buttons: List[Dict[str, pygame.Rect]] = []
    x = WIDTH + 10
    y = 20
    for btn in ACTION_BUTTONS:
        rect = pygame.Rect(x, y, UI_PANEL_WIDTH - 20, 30)
        buttons.append({"id": btn["id"], "label": btn["label"], "rect": rect})
        y += 36
    return buttons


def draw_planner_panel(
    screen: pygame.Surface,
    font: pygame.font.Font,
    state: AppState,
    action_buttons: List[Dict[str, pygame.Rect]],
) -> None:
    panel_rect = pygame.Rect(WIDTH, 0, UI_PANEL_WIDTH, HEIGHT)
    pygame.draw.rect(screen, (245, 245, 245), panel_rect)
    pygame.draw.rect(screen, GRAY, panel_rect, 1)

    header = font.render("Controls", True, (0, 0, 0))
    screen.blit(header, (WIDTH + 10, 4))

    for button in action_buttons:
        rect = button["rect"]
        pygame.draw.rect(screen, (210, 210, 210), rect, border_radius=4)
        pygame.draw.rect(screen, GRAY, rect, 1, border_radius=4)
        label = font.render(button["label"], True, (0, 0, 0))
        screen.blit(label, (rect.x + 8, rect.y + 6))

    # Parameter display
    param_y = action_buttons[-1]["rect"].bottom + 6
    param_lines = [
        f"RRT max_iter: {state.rrt_max_iter}",
        f"PRM samples: {state.prm_samples}",
        f"Hybrid w_len: {state.hybrid_w_length:.2f}",
        f"Hybrid w_safe: {state.hybrid_w_safety:.2f}",
        f"Hybrid L_ref: {state.hybrid_l_ref:.2f}",
        f"Hybrid d_min: {state.hybrid_d_min:.2f}",
        f"Hybrid c_max: {state.hybrid_c_max:.1f}",
    ]
    for line in param_lines:
        surf = font.render(line, True, (0, 0, 0))
        screen.blit(surf, (WIDTH + 10, param_y))
        param_y += surf.get_height() + 2


def render_frame(
    screen: pygame.Surface,
    font: pygame.font.Font,
    state: AppState,
    *,
    visited: Optional[Iterable[Point]] = None,
    tree_lines: Optional[List[Tuple[Point, Point]]] = None,
    action_buttons: Optional[List[Dict[str, pygame.Rect]]] = None,
) -> None:
    screen.fill(WHITE)
    draw_grid(screen, state, visited=visited, tree_lines=tree_lines)
    if action_buttons is not None:
        draw_planner_panel(screen, font, state, action_buttons)
    draw_stats(screen, font, state)
    draw_multi_legend(screen, font, state)
    if state.show_hints:
        draw_hints(screen, font)
    draw_planner_overlay(screen, font, state)
    draw_hybrid_input_overlay(screen, font, state)
    pygame.display.flip()
