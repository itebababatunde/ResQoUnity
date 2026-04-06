# Aerial-Assisted Maze Navigation

## What is this?

A drone hovers high above a maze and looks straight down at it. From that bird's-eye view, it figures out the shortest path from start to finish, then guides a ground robot (the Go2 dog) through the maze step by step.

Neither robot has a map given to it ahead of time. The drone builds the map itself from its camera feed. The dog just follows the directions it receives.

---

## The Two Robots

| Robot | Role |
|---|---|
| **Drone** | Flies up, photographs the maze, plans the route, sends movement commands |
| **Go2 dog** | Starts at the maze entrance and walks the route the drone gives it |

---

## How It Works, Step by Step

**1. Boot up**
Both robots start in Isaac Sim. The drone is on the ground; the dog is placed just outside the maze entrance.

**2. Drone arms and takes off**
The drone arms its motors, then requests a takeoff. It climbs toward the centre of the maze.

**3. Find the right altitude**
The drone doesn't guess how high to go. It starts at 15 m and checks whether all four corners of the maze are visible in its camera frame. If any corner is cut off, it climbs another metre and checks again. It repeats until the whole maze fits in view — typically around 19–20 m for this maze size. Once it passes that check three times in a row, it locks that altitude.

**4. Photograph and map the maze**
The drone hovers and collects 10 camera frames. Each frame is processed with computer vision (grayscale → threshold → morphological cleanup) to identify which areas are walls and which are open passages. The 10 frames are combined by majority vote to produce a clean binary map.

**5. Plan the route**
A\* (a shortest-path algorithm) runs on the map from the dog's starting cell to the goal cell (top-right corner). The raw path is then smoothed to remove unnecessary zigzags. The planned route is drawn on the camera image and displayed for 3 seconds before the dog starts moving.

**6. Guide the dog**
The dog follows the waypoints one by one. The drone continuously sends velocity commands — how fast to go forward and how much to turn. When the dog arrives within 0.3 m of a waypoint, it moves on to the next one.

**7. Recovery if the dog gets stuck**
If the dog stops making progress for 10 seconds, the mission re-enters the perception and planning stages from the dog's current position and gives it a fresh route.

**8. Mission complete**
When the dog reaches the final waypoint, the mission is declared a success. The drone holds position; the dog stops. A report is automatically generated.

---

## Output After Each Run

Everything is saved to the `logs/` folder.

| Output | Location | What it contains |
|---|---|---|
| CSV data log | `logs/maze_<timestamp>.csv` | Position, state, velocity, waypoint progress — every 100 ms across the full mission |
| Camera snapshots | `logs/snapshots/` | PNG images at key stages: survey start, survey locked, perceiving, path planned, each waypoint, mission result |
| Report | `logs/reports/maze_report_<timestamp>.png` | Auto-generated after the mission: trajectory on the maze, state timeline, distance to goal, velocity profile, metrics table |

The report is generated automatically. You don't need to run anything extra.

---

## Running the Scenario

You need two terminals open at the same time.

**Terminal 1 — start the simulation:**
```bash
./run_maze.sh sim
```

Wait until you see the Isaac Sim window open and the maze walls appear. The dog and drone will be visible.

**Terminal 2 — start the coordination node:**
```bash
./run_maze.sh node
```

This is what runs the state machine, controls both robots, saves logs, and generates the report.

**To use a different maze layout**, set the seed before running both commands:
```bash
MAZE_SEED=7 ./run_maze.sh sim
MAZE_SEED=7 ./run_maze.sh node
```

The seed must match in both terminals. Default seed is `42`.

---

## Changing Maze Size

The maze size is set via `MAZE_ROWS` and `MAZE_COLS` environment variables (default 6×6). The drone does not use these values during flight — it discovers the maze position and extent purely from its camera. The variables only affect maze generation and path planning.

**Step 1 — Verify in 2D first (no Isaac Sim needed):**
```bash
MAZE_ROWS=8 MAZE_COLS=8 python3 debug_maze_2d.py --seed 7
```
This opens a plot showing the maze walls and the A* planned path overlaid in orange. Confirm the path runs from the green start marker to the red end marker with no "NO PATH FOUND" warning before proceeding to the sim.

**Step 2 — Run with the larger maze:**
```bash
# Terminal 1
MAZE_ROWS=8 MAZE_COLS=8 MAZE_SEED=7 ./run_maze.sh sim

# Terminal 2
MAZE_ROWS=8 MAZE_COLS=8 MAZE_SEED=7 ./run_maze.sh node
```

Both terminals must use the same values. The drone will automatically climb to whatever altitude is needed to see the full maze — you do not need to adjust any constants.

| Maze size | Approx. locked altitude |
|---|---|
| 6 × 6 (default) | ~20 m |
| 8 × 8 | ~27 m |
| 10 × 10 | ~34 m |

---

## Viewing the Report

After the mission completes, open the report PNG:
```bash
xdg-open logs/reports/maze_report_<timestamp>.png
```

Or to regenerate the report from an old run:
```bash
python3 maze_report.py                             # uses the most recent CSV
python3 maze_report.py logs/maze_<timestamp>.csv   # specific run
```

---

## Key Files

| File | Purpose |
|---|---|
| `run_maze.sh` | Launch script for sim and node |
| `maze_ros2_node.py` | The main brain — state machine, coordinates drone and dog |
| `maze_generator.py` | Procedurally builds the maze at startup |
| `maze_vision.py` | Converts camera images into a wall map |
| `maze_astar.py` | Finds the shortest path through the map |
| `maze_dog_controller.py` | Translates waypoints into velocity commands for the dog |
| `maze_report.py` | Generates the post-mission report PNG |

---

## Maze Specs

- **Size:** N × M cells (default 6×6), 2 m per cell. Set `MAZE_ROWS` / `MAZE_COLS` env vars to change.
- **Total footprint:** `ROWS × COLS × 2 m` (e.g. 12 × 12 m for 6×6, 16 × 16 m for 8×8)
- **Walls:** 0.2 m thick, 0.8 m tall
- **Start:** bottom-left corner of the maze
- **Goal:** top-right corner of the maze
- **Generation:** random (controlled by seed), guaranteed to have exactly one valid route
- **Drone survey:** vision-based — the drone discovers the maze boundary from its camera with no prior knowledge of size or position

---

## ROS2 Topics Used

| Topic | Direction | What it carries |
|---|---|---|
| `/drone/odom` | → node | Drone position and altitude |
| `/robot0/odom` | → node | Dog position and heading |
| `/drone/front_cam/rgb` | → node | Camera image from drone |
| `/drone/cmd_position` | node → | Where the drone should fly to |
| `/robot0/cmd_vel` | node → | Forward speed and turn rate for the dog |
| `/maze/occupancy_grid` | node → | The wall map (for visualisation) |
| `/maze/planned_path` | node → | The planned route (for visualisation) |

Services used: `/drone/arm` to arm motors, `/drone/takeoff` to initiate flight.
