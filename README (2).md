# AI Search Algorithms – Programming Assignments

Three Python implementations of search/pathfinding algorithms: Dijkstra's on India's road network, and UGV navigation on a battlefield grid with static and dynamic obstacles.

---

## Assignment 1 – Dijkstra's Algorithm (India Road Network)

**File:** `assignment1_dijkstra.py`

Implements Dijkstra's algorithm (Uniform-Cost Search) to find shortest road paths between major Indian cities.

**Features:**
- 70+ Indian cities with real approximate road distances
- Find shortest path between any two cities (with segment breakdown)
- Find all shortest paths from a source city
- Interactive menu

**Run:**
```bash
python assignment1_dijkstra.py
```

**Example Output:**
```
Path   : Delhi → Agra → Gwalior → Bhopal → Nagpur → Hyderabad → Chennai
Distance: 2209 km
Hops   : 6
```

---

## Assignment 2 – UGV Navigation with Static Obstacles

**File:** `assignment2_ugv_static.py`

A 70×70 km battlefield grid where an Unmanned Ground Vehicle (UGV) finds the optimal path using **A\*** search. Obstacles are known a-priori (before the UGV moves).

**Features:**
- Three obstacle density levels: LOW (10%), MEDIUM (25%), HIGH (40%)
- 8-directional movement (diagonal cost = √2)
- ASCII grid visualization showing path and obstacles
- Measures of Effectiveness (MOE): path cost, detour factor, nodes explored, runtime

**Run:**
```bash
python assignment2_ugv_static.py
```

**MOE Report includes:**
| Metric | Description |
|---|---|
| Path Cost (km) | Actual travel distance |
| Straight-line (km) | Euclidean distance start→goal |
| Path Optimality | How close to straight-line |
| Nodes Explored | Search efficiency |
| Runtime (ms) | Computation time |

---

## Assignment 3 – UGV Navigation with Dynamic Obstacles

**File:** `assignment3_ugv_dynamic.py`

Extends Assignment 2 to a **dynamic** environment where obstacles are **not known a-priori** — they appear and disappear at runtime. Uses the **D\* Lite** algorithm for efficient incremental replanning.

**Features:**
- Obstacles randomly appear/disappear each step (unknown to UGV)
- Configurable sensor radius (UGV only sees nearby cells)
- Replans only when new obstacles are detected (not a full replan every step)
- Comparison mode: static vs dynamic obstacle environments
- Full MOE report including replanning events and sensor scans

**Run:**
```bash
python assignment3_ugv_dynamic.py
```

**Why D\* Lite?**
> D\* Lite is the industry-standard algorithm for robot/UGV navigation in dynamic environments (used in NASA rovers and military UGVs). It is far more efficient than re-running A\* from scratch after every obstacle change.

---

## Requirements

Python 3.7+ — no external libraries needed (uses only `heapq`, `math`, `random`, `time` from the standard library).

---

## Algorithms Used

| Assignment | Algorithm | Complexity |
|---|---|---|
| 1 | Dijkstra's (Uniform-Cost Search) | O((V + E) log V) |
| 2 | A\* Search | O((V + E) log V) |
| 3 | D\* Lite (Incremental Replanning) | O(k · log V) per replan |

---

## File Structure

```
├── assignment1_dijkstra.py      # Dijkstra on India road network
├── assignment2_ugv_static.py    # UGV + A* with static obstacles
├── assignment3_ugv_dynamic.py   # UGV + D* Lite with dynamic obstacles
└── README.md
```
