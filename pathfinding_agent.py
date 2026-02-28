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
