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

class Button:
    def __init__(self, x, y, w, h, label, toggle=False):
        self.rect   = pygame.Rect(x, y, w, h)
        self.label  = label
        self.toggle = toggle
        self.active = False
        self.font   = pygame.font.SysFont("Arial", 13)

    def draw(self, screen):
        if self.active and self.toggle:
            color = BTN_ON
        elif self.rect.collidepoint(pygame.mouse.get_pos()):
            color = BTN_HOVER
        else:
            color = BTN_NORMAL
        pygame.draw.rect(screen, color, self.rect, border_radius=5)
        pygame.draw.rect(screen, GRAY,  self.rect, 1, border_radius=5)
        t = self.font.render(self.label, True, WHITE)
        screen.blit(t, t.get_rect(center=self.rect.center))

    def clicked(self, event):
        return (event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 1
                and self.rect.collidepoint(event.pos))

class App:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Dynamic Pathfinding Agent")
        self.font_s = pygame.font.SysFont("Arial", 13)
        self.font_m = pygame.font.SysFont("Arial", 15, bold=True)
        self.font_l = pygame.font.SysFont("Arial", 19, bold=True)

        self.rows  = 24
        self.cols  = 33
        self.walls = set()
        self.start = (0, 0)
        self.goal  = (self.rows-1, self.cols-1)
        self.path     = []
        self.visited  = set()
        self.frontier = []
        self.m_nodes = 0
        self.m_cost  = 0
        self.m_time  = 0.0
        self.algorithm = "A*"
        self.heuristic = "Manhattan"
        self.draw_mode = None
        self.dynamic_on    = False
        self.agent_running = False
        self.agent_pos     = None
        self.agent_step    = 0
        self.spawn_prob    = 0.04
        self.status = "generate a map or draw walls, then click run search."
        self._make_buttons()
        self._generate_map()

    def _make_buttons(self):
        x  = WINDOW_W - SIDEBAR_W + 10
        bw = SIDEBAR_W - 20
        bh = 30
        def B(label, y, toggle=False):
            return Button(x, y, bw, bh, label, toggle)
        self.btn_gen     = B("Generate Random Map",  70)
        self.btn_clear   = B("Clear Walls",         108)
        self.btn_wall    = B("Draw Walls",   155, toggle=True)
        self.btn_erase   = B("Erase Walls",  193, toggle=True)
        self.btn_start   = B("Set Start",    231, toggle=True)
        self.btn_goal    = B("Set Goal",     269, toggle=True)
        self.btn_astar   = B("A*",           318, toggle=True)
        self.btn_gbfs    = B("GBFS",         356, toggle=True)
        self.btn_astar.active = True
        self.btn_manh    = B("Manhattan",    404, toggle=True)
        self.btn_eucl    = B("Euclidean",    442, toggle=True)
        self.btn_manh.active = True
        self.btn_run     = B("Run Search",   490)
        self.btn_dynamic = B("Dynamic Mode", 528, toggle=True)
        self.btn_play    = B("Play Agent",   566)
        self.btn_reset   = B("Reset View",   604)
        self.buttons = [
            self.btn_gen, self.btn_clear,
            self.btn_wall, self.btn_erase, self.btn_start, self.btn_goal,
            self.btn_astar, self.btn_gbfs,
            self.btn_manh, self.btn_eucl,
            self.btn_run, self.btn_dynamic, self.btn_play, self.btn_reset,
        ]

    def _generate_map(self, density=0.30):
        self.walls = set()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in (self.start, self.goal):
                    if random.random() < density:
                        self.walls.add((r,c))
        self._reset_view()
        self.status = "New map generated."

    def _reset_view(self):
        self.path = []
        self.visited  = set()
        self.frontier = []
        self.agent_running = False
        self.agent_pos     = None
        self.agent_step    = 0

    def _draw_grid(self):
        path_set = set(self.path)
        front_set = set(self.frontier)
        for r in range(self.rows):
            for c in range(self.cols):
                cell = (r, c)
                rect = pygame.Rect(c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                if cell in self.walls:          color = COLOR_WALL
                elif cell == self.start:        color = COLOR_START
                elif cell == self.goal:         color = COLOR_GOAL
                elif cell == self.agent_pos:    color = COLOR_AGENT
                elif cell in path_set:          color = COLOR_PATH
                elif cell in front_set:         color = COLOR_FRONTIER
                elif cell in self.visited:      color = COLOR_VISITED
                else:                           color = COLOR_EMPTY
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, GRAY,  rect, 1)
        for cell, lbl in [(self.start,"S"), (self.goal,"G")]:
            r, c = cell
            rect = pygame.Rect(c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            t = self.font_s.render(lbl, True, WHITE)
            self.screen.blit(t, t.get_rect(center=rect.center))

    def _draw_sidebar(self):
        px = WINDOW_W - SIDEBAR_W
        pygame.draw.rect(self.screen, PANEL_BG, pygame.Rect(px, 0, SIDEBAR_W, WINDOW_H))
        def txt(msg, y, color=TEXT_WHITE, font=None):
            f = font or self.font_s
            self.screen.blit(f.render(msg, True, color), (px+10, y))
        txt("Pathfinding Agent", 10, color=(80,200,255), font=self.font_l)
        txt("Map",       50,  font=self.font_m)
        txt("Tools",    140,  font=self.font_m)
        txt("Algorithm", 305, font=self.font_m)
        txt("Heuristic", 390, font=self.font_m)
        txt("Run",       477, font=self.font_m)
        for btn in self.buttons:
            btn.draw(self.screen)
        txt("Metrics", 645, color=(80,200,255), font=self.font_m)
        txt(f"Nodes expanded : {self.m_nodes}", 668)
        txt(f"Path cost      : {self.m_cost if self.m_cost >= 0 else 'N/A'}", 686)
        txt(f"Time (ms)      : {self.m_time:.2f}", 704)
        txt(f"Algorithm      : {self.algorithm}", 722)
        legend = [
            (COLOR_START, "Start"), (COLOR_GOAL, "Goal"), (COLOR_PATH, "Path"),
            (COLOR_FRONTIER, "Frontier (queue)"), (COLOR_VISITED, "Visited"),
            (COLOR_WALL, "Wall"), (COLOR_AGENT, "Agent"),
        ]
        lx, ly = px+10, WINDOW_H - 170
        txt("── Legend ──", ly-18, color=(80,200,255), font=self.font_m)
        for color, name in legend:
            pygame.draw.rect(self.screen, color, pygame.Rect(lx, ly, 14, 14), border_radius=3)
            txt(name, ly, color=TEXT_WHITE)
            lx += 18 + self.font_s.size(name)[0] + 12
            if lx > WINDOW_W - 20:
                lx = px + 10
                ly += 18
        txt("Status: " + self.status, WINDOW_H - 22, color=(240,220,80))

    def run(self):
        clock = pygame.time.Clock()
        while True:
            clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
            self.screen.fill(WHITE)
            self._draw_grid()
            self._draw_sidebar()
            pygame.display.flip()

if __name__ == "__main__":
    App().run()
