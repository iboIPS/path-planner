import time
from math import sqrt
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


def _lazy_dijkstra(
    graph: Dict[Point, List[Tuple[Point, float]]],
    grid: Grid,
    start: Point,
    goal: Point,
    validated_edges: Set[Tuple[Point, Point]],
    invalid_edges: Set[Tuple[Point, Point]],
) -> Optional[List[Point]]:
    """Dijkstra that validates edges lazily during search"""
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
            # Validate entire path
            path = []
            node = current
            while node:
                path.append(node)
                node = previous[node]
            path.reverse()
            
            # Check all edges in path
            for i in range(len(path) - 1):
                edge = (path[i], path[i+1])
                if edge not in validated_edges:
                    if _line_collision(grid, path[i], path[i+1]):
                        invalid_edges.add(edge)
                        invalid_edges.add((path[i+1], path[i]))
                        # Path invalid, need to search again
                        return None
                    validated_edges.add(edge)
                    validated_edges.add((path[i+1], path[i]))
            
            return path
        
        for neighbor, edge_cost in graph.get(current, []):
            if neighbor in visited:
                continue
            
            edge = (current, neighbor)
            if edge in invalid_edges:
                continue
            
            # Lazy evaluation: only check collision if this becomes part of solution
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
    connection_radius: float = 10.0,
    max_neighbors: int = 10,
    max_retries: int = 5,
    animate: bool = False,
    visualize: Visualizer = None,
) -> tuple[List[Point], int]:
    """Lazy PRM: delays collision checking until query time"""
    start_time = time.time()
    grid_size = len(grid)
    
    # Phase 1: Construction - Build roadmap WITHOUT collision checking
    nodes: List[Point] = [start, goal]
    graph: Dict[Point, List[Tuple[Point, float]]] = {start: [], goal: []}
    prm_lines: List[Tuple[Point, Point]] = []
    
    # Sample random points (only check if point itself is collision-free)
    attempts = 0
    while len(nodes) < num_samples + 2 and attempts < num_samples * 3:
        attempts += 1
        sample = (randint(0, grid_size - 1), randint(0, grid_size - 1))
        
        if grid[sample[1]][sample[0]] == 1:
            continue
        
        if sample not in nodes:
            nodes.append(sample)
            graph[sample] = []
    
    # Connect nodes within radius WITHOUT collision checking edges
    for i, node in enumerate(nodes):
        neighbors = []
        for j, other in enumerate(nodes):
            if i != j:
                dist = _distance(node, other)
                if dist <= connection_radius:
                    neighbors.append((dist, other))
        
        neighbors.sort()
        neighbors = neighbors[:max_neighbors]
        
        for dist, neighbor in neighbors:
            graph[node].append((neighbor, dist))
            graph.setdefault(neighbor, []).append((node, dist))
            prm_lines.append((node, neighbor))
            if animate and visualize:
                visualize(prm_lines)
    
    # Phase 2: Query - Find path with lazy collision checking
    validated_edges: Set[Tuple[Point, Point]] = set()
    invalid_edges: Set[Tuple[Point, Point]] = set()
    
    for attempt in range(max_retries):
        path = _lazy_dijkstra(graph, grid, start, goal, validated_edges, invalid_edges)
        
        if path:
            duration_ms = int((time.time() - start_time) * 1000)
            dense: List[Point] = []
            for i in range(len(path) - 1):
                seg = _interpolate(path[i], path[i + 1])
                if dense:
                    dense.extend(seg[1:])
                else:
                    dense.extend(seg)
            return dense or path, duration_ms, len(nodes)
        
        # If path found but invalid, search again (invalid edges are now marked)
    
    duration_ms = int((time.time() - start_time) * 1000)
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
