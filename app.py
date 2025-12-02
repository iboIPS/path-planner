import sys
from typing import Callable, Iterable, List, Optional, Tuple

import pygame

from src.config import CELL_SIZE, GRID_SIZE
from src.planners import PLANNERS
from src.render import (
    init_display,
    layout_action_buttons,
    render_frame,
)
from src.state import AppState

Point = Tuple[int, int]


def _apply_hybrid_input(state: AppState) -> None:
    text = state.hybrid_input_text.strip()
    if not text:
        state.hybrid_input_active = False
        return
    try:
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 5:
            print("Enter 5 comma-separated values for w_len, w_safe, L_ref, d_min, c_max")
            return
        w_len, w_safe, l_ref, d_min, c_max = [float(p) for p in parts]
        state.hybrid_w_length = max(0.0, round(w_len, 3))
        state.hybrid_w_safety = max(0.0, round(w_safe, 3))
        state.hybrid_l_ref = max(0.01, round(l_ref, 3))
        state.hybrid_d_min = max(0.0, round(d_min, 3))
        state.hybrid_c_max = max(0.0, round(c_max, 3))
    except Exception as exc:
        print(f"Could not parse HybridSafety params: {exc}")
    finally:
        state.hybrid_input_active = False


def _start_planner_overlay(state: AppState) -> None:
    state.planner_menu = list(PLANNERS.keys())
    state.planner_cursor = min(state.planner_cursor, max(0, len(state.planner_menu) - 1))
    state.planner_overlay = True


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
        if state.planner_mode == "HybridSafety":
            extra_kwargs.update(
                {
                    "w_length": state.hybrid_w_length,
                    "w_safety": state.hybrid_w_safety,
                    "l_ref": state.hybrid_l_ref,
                    "d_min": state.hybrid_d_min,
                    "c_max": state.hybrid_c_max,
                }
            )

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
    if state.planner_overlay:
        if event.key == pygame.K_p:
            state.planner_overlay = False
            return
        if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            if state.planner_menu:
                state.planner_mode = state.planner_menu[state.planner_cursor]
            state.planner_overlay = False
        elif event.key == pygame.K_ESCAPE:
            state.planner_overlay = False
        elif event.key == pygame.K_DOWN:
            if state.planner_menu:
                state.planner_cursor = (state.planner_cursor + 1) % len(state.planner_menu)
        elif event.key == pygame.K_UP:
            if state.planner_menu:
                state.planner_cursor = (state.planner_cursor - 1) % len(state.planner_menu)
        return

    if state.hybrid_input_active:
        if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            _apply_hybrid_input(state)
            return
        if event.key == pygame.K_ESCAPE:
            state.hybrid_input_active = False
            return
        if event.key == pygame.K_BACKSPACE:
            state.hybrid_input_text = state.hybrid_input_text[:-1]
            return
        ch = event.unicode
        if ch and ch in "0123456789.-, ":
            state.hybrid_input_text += ch
        return

    if event.key == pygame.K_p:
        _start_planner_overlay(state)
        return
    if event.key == pygame.K_i:
        state.hybrid_input_active = True
        state.hybrid_input_text = (
            f"{state.hybrid_w_length}, {state.hybrid_w_safety}, "
            f"{state.hybrid_l_ref}, {state.hybrid_d_min}, {state.hybrid_c_max}"
        )
        return
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
    drawing = False
    draw_mode = 1

    while True:
        render_frame(
            screen,
            font,
            state,
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
                            elif btn["id"] == "choose_planner":
                                _start_planner_overlay(state)
                            elif btn["id"] == "rrt_inc":
                                state.rrt_max_iter = min(state.rrt_max_iter + 100, 10000)
                            elif btn["id"] == "rrt_dec":
                                state.rrt_max_iter = max(state.rrt_max_iter - 100, 100)
                            elif btn["id"] == "prm_inc":
                                state.prm_samples = min(state.prm_samples + 100, 5000)
                            elif btn["id"] == "prm_dec":
                                state.prm_samples = max(state.prm_samples - 100, 50)
                            elif btn["id"] == "hybrid_len_inc":
                                state.hybrid_w_length = round(state.hybrid_w_length + 0.1, 2)
                            elif btn["id"] == "hybrid_len_dec":
                                state.hybrid_w_length = max(0.0, round(state.hybrid_w_length - 0.1, 2))
                            elif btn["id"] == "hybrid_safe_inc":
                                state.hybrid_w_safety = round(state.hybrid_w_safety + 0.1, 2)
                            elif btn["id"] == "hybrid_safe_dec":
                                state.hybrid_w_safety = max(0.0, round(state.hybrid_w_safety - 0.1, 2))
                            elif btn["id"] == "hybrid_dmin_inc":
                                state.hybrid_d_min = round(state.hybrid_d_min + 0.5, 2)
                            elif btn["id"] == "hybrid_dmin_dec":
                                state.hybrid_d_min = max(0.0, round(state.hybrid_d_min - 0.5, 2))
                            elif btn["id"] == "hybrid_cmax_inc":
                                state.hybrid_c_max = min(5000.0, state.hybrid_c_max + 100.0)
                            elif btn["id"] == "hybrid_cmax_dec":
                                state.hybrid_c_max = max(0.0, state.hybrid_c_max - 100.0)
                            elif btn["id"] == "hybrid_lref_inc":
                                state.hybrid_l_ref = round(state.hybrid_l_ref + 0.5, 2)
                            elif btn["id"] == "hybrid_lref_dec":
                                state.hybrid_l_ref = max(0.1, round(state.hybrid_l_ref - 0.5, 2))
                            elif btn["id"] == "hybrid_set":
                                state.hybrid_input_active = True
                                state.hybrid_input_text = (
                                    f"{state.hybrid_w_length}, {state.hybrid_w_safety}, "
                                    f"{state.hybrid_l_ref}, {state.hybrid_d_min}, {state.hybrid_c_max}"
                                )
                            handled = True
                            break
                    if handled:
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
