import sys
from typing import Callable, Iterable, List, Optional, Tuple

import pygame
from tkinter import filedialog, Tk

from src.config import CELL_SIZE, GRID_SIZE
from src.planners import PLANNERS
from src.render import (
    init_display,
    layout_action_buttons,
    layout_planner_buttons,
    render_frame,
)
from src.state import AppState

Point = Tuple[int, int]


def _draw_and_wait(
    screen: pygame.Surface,
    font: pygame.font.Font,
    state: AppState,
    *,
    visited: Optional[Iterable[Point]] = None,
    tree_lines: Optional[List[Tuple[Point, Point]]] = None,
) -> None:
    render_frame(screen, font, state, visited=visited, tree_lines=tree_lines)
    pygame.time.wait(state.animation_delay)


def build_visualizer(
    state: AppState,
    screen: pygame.Surface,
    font: pygame.font.Font,
) -> Optional[Callable[[Iterable[Point]], None]]:
    if not state.animate:
        return None

    tree_planners = {
        "RRT",
        "RRT*",
        "RRTConnect",
        "RRTDynamic",
        "RRTInformed*",
        "RRTGoalBiased",
        "PRM",
        "PRM*",
        "PRMLazy",
        "PRMGuassian",
        "PRMBridge",
    }
    if state.planner_mode in tree_planners:
        return lambda lines: _draw_and_wait(screen, font, state, tree_lines=lines)
    return lambda visited: _draw_and_wait(screen, font, state, visited=visited)


def run_planner(
    state: AppState,
    screen: pygame.Surface,
    font: pygame.font.Font,
) -> None:
    if not state.has_start_and_goal:
        return

    state.planner_status = "running"
    planner_fn = PLANNERS[state.planner_mode]
    visualize = build_visualizer(state, screen, font)

    if state.planner_mode == "MultiPaths":
        results, duration_ms = planner_fn(
            state.grid,
            state.start_pos,  # type: ignore[arg-type]
            state.goal_pos,  # type: ignore[arg-type]
            paths_count=state.multi_count,
            animate=state.animate,
            visualize=visualize,
        )
        state.update_multi_paths(results, duration_ms)
    else:
        extra_kwargs: dict = {"animate": state.animate, "visualize": visualize}
        if "RRT" in state.planner_mode:
            extra_kwargs["max_iter"] = state.rrt_max_iter
        if "PRM" in state.planner_mode:
            extra_kwargs["num_samples"] = state.prm_samples

        result = planner_fn(
            state.grid,
            state.start_pos,  # type: ignore[arg-type]
            state.goal_pos,  # type: ignore[arg-type]
            **extra_kwargs,
        )
        if isinstance(result, tuple) and len(result) == 3:
            path, duration_ms, iteration = result
        else:
            path, duration_ms = result  # type: ignore[misc]
            iteration = None
        state.update_path(path, duration_ms, iteration)
    state.planner_status = "done" if (state.path or state.multi_paths) else "failed"


def handle_keydown(
    event: pygame.event.Event,
    state: AppState,
    screen: pygame.Surface,
    font: pygame.font.Font,
) -> None:
    if event.key == pygame.K_SPACE:
        run_planner(state, screen, font)
    elif event.key == pygame.K_r:
        state.reset_grid()
    elif event.key == pygame.K_u:
        if event.mod & pygame.KMOD_CTRL:
            state.undo()
        else:
            state.planner_mode = "MultiPaths"
    elif event.key == pygame.K_y:
        state.redo()
    elif event.key == pygame.K_a:
        state.animate = not state.animate
        print(f"A* Animation: {'ON' if state.animate else 'OFF'}")
    elif event.key == pygame.K_m:
        state.generate_random_map()
    elif event.key == pygame.K_g:
        state.generate_random_start_goal()
    elif event.key == pygame.K_1:
        state.planner_mode = "A*"
    elif event.key == pygame.K_2:
        state.planner_mode = "D*"
    elif event.key == pygame.K_3:
        state.planner_mode = "RRT"
    elif event.key == pygame.K_4:
        state.planner_mode = "Dijkstra"
    elif event.key == pygame.K_5:
        state.planner_mode = "RRT*"
    elif event.key == pygame.K_6:
        state.planner_mode = "PRM"
    elif event.key == pygame.K_7:
        state.planner_mode = "DWA"
    elif event.key == pygame.K_8:
        state.planner_mode = "TEB"
    elif event.key == pygame.K_9:
        state.planner_mode = "VFH"
    elif event.key == pygame.K_0:
        state.planner_mode = "PotentialField"
    elif event.key == pygame.K_UP:
        state.multi_count = min(state.multi_count + 1, 50)
        print(f"MultiPaths count: {state.multi_count}")
    elif event.key == pygame.K_DOWN:
        state.multi_count = max(state.multi_count - 1, 1)
        print(f"MultiPaths count: {state.multi_count}")
    elif event.key == pygame.K_h:
        state.show_hints = not state.show_hints
    elif event.key == pygame.K_LEFTBRACKET:  # [
        state.animation_delay = min(state.animation_delay + 5, 200)
        print(f"Animation delay: {state.animation_delay} ms")
    elif event.key == pygame.K_RIGHTBRACKET:  # ]
        state.animation_delay = max(state.animation_delay - 5, 0)
        print(f"Animation delay: {state.animation_delay} ms")


def main() -> None:
    screen, font = init_display()
    state = AppState(GRID_SIZE, CELL_SIZE)
    action_buttons = layout_action_buttons(font)
    planner_buttons = layout_planner_buttons(font, start_y=action_buttons[-1]["rect"].bottom + 70)
    drawing = False
    draw_mode = 1

    while True:
        render_frame(
            screen,
            font,
            state,
            planner_buttons=planner_buttons,
            action_buttons=action_buttons,
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_pos = pygame.mouse.get_pos()
                    # Check action buttons first
                    handled = False
                    for btn in action_buttons:
                        if btn["rect"].collidepoint(mouse_pos):
                            if btn["id"] == "simulate":
                                state.animate = True
                                run_planner(state, screen, font)
                            elif btn["id"] == "random_goal":
                                state.generate_random_start_goal()
                            elif btn["id"] == "random_map":
                                state.generate_random_map()
                            elif btn["id"] == "load_image":
                                import subprocess
                                result = subprocess.run([
                                    'osascript', '-e',
                                    'tell application "System Events" to activate',
                                    '-e', 'POSIX path of (choose file with prompt "Select map image" of type {"public.image"})'
                                ], capture_output=True, text=True)
                                
                                if result.returncode == 0:
                                    file_path = result.stdout.strip()
                                    if file_path:
                                        state.load_map_from_image(file_path)
                            elif btn["id"] == "rrt_inc":
                                state.rrt_max_iter = min(state.rrt_max_iter + 100, 10000)
                            elif btn["id"] == "rrt_dec":
                                state.rrt_max_iter = max(state.rrt_max_iter - 100, 100)
                            elif btn["id"] == "prm_inc":
                                state.prm_samples = min(state.prm_samples + 100, 5000)
                            elif btn["id"] == "prm_dec":
                                state.prm_samples = max(state.prm_samples - 100, 50)
                            handled = True
                            break
                    if handled:
                        continue
                    # Check planner buttons
                    clicked_planner = False
                    for btn in planner_buttons:
                        if btn["rect"].collidepoint(mouse_pos):
                            state.planner_mode = btn["name"]
                            clicked_planner = True
                            break
                    if clicked_planner:
                        continue
                    drawing = True
                    draw_mode = 1
                    state.set_cell_at_pixel(mouse_pos, draw_mode)
                elif event.button == 3:
                    drawing = True
                    draw_mode = 0
                    state.set_cell_at_pixel(pygame.mouse.get_pos(), draw_mode)
                elif event.button == 2:
                    state.set_start_or_goal_at_pixel(pygame.mouse.get_pos())

            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False

            elif event.type == pygame.MOUSEMOTION and drawing:
                state.set_cell_at_pixel(pygame.mouse.get_pos(), draw_mode)

            elif event.type == pygame.KEYDOWN:
                handle_keydown(event, state, screen, font)


if __name__ == "__main__":
    main()
