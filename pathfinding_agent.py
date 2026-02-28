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
