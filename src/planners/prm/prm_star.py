import time
from math import sqrt, log, pi
from random import randint
from typing import Callable, List, Optional, Tuple, Dict, Set
import heapq

Point = Tuple[int, int]
Grid = List[List[int]]
Visualizer = Optional[Callable[[List[Tuple[Point, Point]]], None]]


def _distance(a: Point, b: Point) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _line_collision(grid: Grid, p1: Point, p2: Point) -> bool:
    """Check if line between two points collides with obstacles"""
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if not (0 <= x < len(grid) and 0 <= y < len(grid)):
            return True
        if grid[y][x] == 1:
            return True
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return False


def _dijkstra(graph: Dict[Point, List[Tuple[Point, float]]], start: Point, goal: Point) -> Optional[List[Point]]:
    """Find shortest path in graph using Dijkstra's algorithm"""
    distances: Dict[Point, float] = {start: 0}
    previous: Dict[Point, Optional[Point]] = {start: None}
    pq = [(0, start)]
    visited: Set[Point] = set()
    
    while pq:
        current_dist, current = heapq.heappop(pq)
        
        if current in visited:
            continue
        
        visited.add(current)
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = previous[current]
            return path[::-1]
        
        for neighbor, edge_cost in graph.get(current, []):
            if neighbor in visited:
                continue
            
            new_dist = current_dist + edge_cost
            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(pq, (new_dist, neighbor))
    
    return None


def run(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    num_samples: int = 500,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    """PRM*: asymptotically optimal version with adaptive connection radius"""
    start_time = time.time()
    grid_size = len(grid)
    
    # Phase 1: Construction - Build roadmap
    nodes: List[Point] = [start, goal]
    graph: Dict[Point, List[Tuple[Point, float]]] = {start: [], goal: []}
    prm_lines: List[Tuple[Point, Point]] = []
    
    # Sample random collision-free points
    attempts = 0
    while len(nodes) < num_samples + 2 and attempts < num_samples * 3:
        attempts += 1
        sample = (randint(0, grid_size - 1), randint(0, grid_size - 1))
        
        if grid[sample[1]][sample[0]] == 1:
            continue
        
        if sample not in nodes:
            nodes.append(sample)
            graph[sample] = []
    
    # PRM* connection radius formula for asymptotic optimality
    # r_n = gamma * (log(n) / n)^(1/d) where d is dimension (2 for 2D)
    n = len(nodes)
    d = 2
    gamma = 2 * (1 + 1.0/d) ** (1.0/d) * (grid_size / pi) ** (1.0/d)
    connection_radius = gamma * ((log(n) / n) ** (1.0/d)) if n > 1 else grid_size
    connection_radius = max(connection_radius, 5.0)  # Minimum radius
    
    # Connect nodes within connection radius
    for i, node in enumerate(nodes):
        neighbors = []
        for j, other in enumerate(nodes):
            if i != j:
                dist = _distance(node, other)
                if dist <= connection_radius:
                    neighbors.append((dist, other))
        
        # Add collision-free edges (all neighbors within radius for PRM*)
        for dist, neighbor in neighbors:
            if not _line_collision(grid, node, neighbor):
                # Add bidirectional edges
                if (neighbor, dist) not in graph[node]:
                    graph[node].append((neighbor, dist))
                if (node, dist) not in graph[neighbor]:
                    graph[neighbor].append((node, dist))
                
                prm_lines.append((node, neighbor))
                
                if animate and visualize:
                    visualize(prm_lines)
    
    # Phase 2: Query - Find path in roadmap
    path = _dijkstra(graph, start, goal)
    
    duration_ms = int((time.time() - start_time) * 1000)
    
    if path:
        dense: List[Point] = []
        for i in range(len(path) - 1):
            seg = _interpolate(path[i], path[i + 1])
            if dense:
                dense.extend(seg[1:])
            else:
                dense.extend(seg)
        return dense or path, duration_ms, len(nodes)
    return [], duration_ms, len(nodes)


def _interpolate(a: Point, b: Point) -> List[Point]:
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    pts = [(x, y)]
    if dx > dy:
        err = dx // 2
        while x != x1:
            x += sx
            err -= dy
            if err < 0:
                y += sy
                err += dx
            pts.append((x, y))
    else:
        err = dy // 2
        while y != y1:
            y += sy
            err -= dx
            if err < 0:
                x += sx
                err += dy
            pts.append((x, y))
    return pts
