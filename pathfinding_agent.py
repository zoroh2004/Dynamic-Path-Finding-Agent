"""
Dynamic Pathfinding Agent
Algorithms: A* GBFS
Heuristics: Manhattan and Euclidean distance
GUI: Pygame
"""
import pygame, heapq, math, random, time, sys

WINDOW_W  = 1100
WINDOW_H  = 750
SIDEBAR_W = 260
CELL_SIZE = 24

WHITE      = (255, 255, 255)
GRAY       = (200, 200, 200)
COLOR_EMPTY    = (245, 245, 245)
COLOR_WALL     = (40,  40,  40)
COLOR_START    = (50,  200,  80)
COLOR_GOAL     = (220,  50,  50)
COLOR_VISITED  = (180, 130, 230)
COLOR_FRONTIER = (250, 210,  50)
COLOR_PATH     = (70,  180, 240)
COLOR_AGENT    = (255, 140,   0)

PANEL_BG   = (25,  35,  50)
TEXT_WHITE = (220, 230, 240)
BTN_NORMAL = (60,  80, 110)
BTN_HOVER  = (80, 100, 140)
BTN_ON     = (50, 180,  80)

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def get_neighbors(cell, rows, cols, walls):
    r, c = cell
    neighbors = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and (nr,nc) not in walls:
            neighbors.append((nr, nc))
    return neighbors

def build_path(came_from, start, goal):
    if goal not in came_from:
        return []
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path if path[0] == start else []

def greedy_bfs(start, goal, rows, cols, walls, h):
    t0 = time.perf_counter()
    open_list  = [(h(start, goal), start)]
    came_from  = {start: None}
    visited    = set()
    count      = 0

    while open_list:
        _, current = heapq.heappop(open_list)
        if current in visited:
            continue
        visited.add(current)
        count += 1
        if current == goal:
            break
        for nb in get_neighbors(current, rows, cols, walls):
            if nb not in visited and nb not in came_from:
                came_from[nb] = current
                heapq.heappush(open_list, (h(nb, goal), nb))

    ms       = (time.perf_counter() - t0) * 1000
    frontier = [cell for _, cell in open_list]
    return build_path(came_from, start, goal), visited, frontier, count, ms

def astar(start, goal, rows, cols, walls, h):
    t0 = time.perf_counter()
    open_list  = [(0, start)]
    came_from  = {start: None}
    g_cost     = {start: 0}
    visited    = set()
    count      = 0

    while open_list:
        _, current = heapq.heappop(open_list)
        if current in visited:
            continue
        visited.add(current)
        count += 1
        if current == goal:
            break
        for nb in get_neighbors(current, rows, cols, walls):
            new_g = g_cost[current] + 1
            if nb not in g_cost or new_g < g_cost[nb]:
                g_cost[nb]    = new_g
                came_from[nb] = current
                heapq.heappush(open_list, (new_g + h(nb, goal), nb))

    ms       = (time.perf_counter() - t0) * 1000
    frontier = [cell for _, cell in open_list]
    return build_path(came_from, start, goal), visited, frontier, count, ms
