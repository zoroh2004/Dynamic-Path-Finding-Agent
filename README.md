# Dynamic-Path-Finding-Agent


# Dynamic Pathfinding Agent

A fully interactive GUI-based pathfinding visualizer implementing **Greedy Best-First Search (GBFS)** and **A\*** with Manhattan and Euclidean heuristics, dynamic obstacle spawning, and real-time re-planning.

---

## Features

| Feature | Details |
|---|---|
| Algorithms | A\* and Greedy Best-First Search (GBFS) |
| Heuristics | Manhattan Distance, Euclidean Distance |
| Grid | User-defined rows × cols, resizable window |
| Map Generation | Random maze with configurable wall density |
| Map Editor | Click/drag to draw or erase walls; set Start/Goal |
| Dynamic Mode | Obstacles spawn mid-navigation; agent re-plans instantly |
| Metrics | Nodes visited, path cost, execution time (ms) |
| Visualization | Frontier (yellow), Visited (purple), Path (blue) |

---

## Requirements

- Python 3.8+
- Pygame

## Installation

```bash
pip install pygame
```

## Running

### Default (25×35 grid, 30% walls)
```bash
python pathfinding_agent.py
```

### Interactive setup prompt
```bash
python pathfinding_agent.py
# Enter rows, cols, density when prompted
```

### Command-line arguments
```bash
python pathfinding_agent.py <rows> <cols> <density>
# Example:
python pathfinding_agent.py 20 30 0.25
```

---

## Controls

| Action | How |
|---|---|
| Generate new map | Click **Generate Map** |
| Draw walls | Click **Draw Walls**, then click/drag on grid |
| Erase walls | Click **Erase Walls**, then click/drag |
| Set Start | Click **Set Start**, then click a cell |
| Set Goal | Click **Set Goal**, then click a cell |
| Run search | Click **Run Search** (or press **R**) |
| Play dynamic agent | Enable **Dynamic Mode**, click **Play / Step** |
| Reset visuals | Click **Reset Visuals** |
| Exit draw mode | Press **Escape** |
| New map (keyboard) | Press **G** |

---

## Color Legend

| Color | Meaning |
|---|---|
| 🟩 Green | Start node |
| 🟥 Red | Goal node |
| 🟨 Yellow | Frontier (open list) |
| 🟪 Purple | Visited (expanded) |
| 🔵 Blue | Final path |
| ⬛ Black | Wall |
| 🟠 Orange | Agent (dynamic mode) |

---

## Algorithm Notes

### A\* — `f(n) = g(n) + h(n)`
Guarantees the optimal path. Explores less than BFS in practice thanks to the heuristic guide, but more than GBFS when obstacles create detours.

### Greedy Best-First Search — `f(n) = h(n)`
Very fast in open maps, but **not** guaranteed optimal. Can get "trapped" in dead ends and require significant backtracking.

---

## Dynamic Re-planning

When **Dynamic Mode** is enabled and **Play** is clicked:
1. The agent moves step-by-step along the computed path.
2. At each step, a new wall may spawn (default probability: 3%).
3. If the wall falls on the remaining path, the agent **immediately re-runs the selected algorithm** from its current position.
4. If no path exists, the agent halts and reports "Path blocked."
