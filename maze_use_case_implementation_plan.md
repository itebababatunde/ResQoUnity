# Maze Navigation Use Case — Implementation Plan

## Context
The goal is to demonstrate aerial-assisted maze navigation for a thesis: a drone hovers high above a maze, derives a map from its downward camera feed, computes the shortest path with A*, and guides a single Go2 dog through the maze via waypoints. This is a new scenario distinct from the existing lawnmower/coverage mission. The implementation reuses the existing drone controller, ROS2 bridge, and coordination node patterns, adding a vision pipeline, path planner, and maze environment.

**User requirements confirmed:**
- Map is camera-derived (OpenCV processing of drone RGB feed)
- Maze is procedurally generated at runtime (USD box prims)
- 1 dog only
- Drone hovers fixed at altitude — does not track the dog

---

## Capacity Assessment (Gaps vs Existing)

| Requirement | Status | Gap |
|---|---|---|
| Drone leader, dog follower ROS2 | ✅ Exists | Pattern reusable |
| Downward camera on drone | ✅ Exists (`/robot{i}/front_cam/rgb`) | Camera graph for world drone needed |
| Drone POSITION/LOITER flight modes | ✅ Exists | No change needed |
| Dog velocity commands `/robot0/cmd_vel` | ✅ Exists | No change needed |
| Nav2 stack / ROS2 bridge | ✅ Exists | No change needed |
| Maze USD asset | ❌ Missing | Must generate procedurally |
| Occupancy grid / map representation | ❌ Missing | Build from camera |
| A* / shortest path planner | ❌ Missing | Must implement |
| Vision pipeline (image → grid) | ❌ Missing | Must implement |
| Waypoint follower for dog | ❌ Missing | Must implement |
| Coordination state machine | ❌ Missing (lawnmower-specific) | Must implement |
| World drone camera OmniGraph | ❌ Missing | ~15 lines in omnigraph.py |

---

## Drone Altitude Calculation

Camera params (from `ros2_bridge.py`): `focal_length=24.0`, `horizontal_aperture=20.955`, `640x480`.

```
FOV_h = 2 * atan(20.955 / 48.0) ≈ 47.2°
FOV_v ≈ 36.2°  (from pixel aspect ratio)
To frame 6m maze vertically: altitude = 3.0 / tan(18.1°) ≈ 9.2m
Target with 15% margin: 10.5m
```

At 10.5m: visible footprint ≈ 10.7m × 8.0m — fully contains a 6×6m maze.

---

## New Files to Create

### 1. `maze_generator.py`
Procedural maze via recursive-backtracking (DFS). Spawns walls as USD `Cube` prims with `UsdPhysics.CollisionAPI` (kinematic). Maze: 6×6 cells, 1m cell size, 0.1m wall thickness, 0.4m wall height, centered at origin.

Key methods:
- `generate_maze_grid(rows, cols)` — returns 2D grid with N/E/S/W wall flags
- `spawn_walls_in_stage(grid, stage)` — creates USD prims at correct world positions
- `get_occupancy_grid()` — returns `(13×13)` binary array (expanded grid format)
- `get_cell_center_world(row, col)` — returns (x, y) world coordinates
- `spawn_markers(stage)` — green cylinder at start (0,0), red cylinder at end (5,5)

### 2. `maze_vision.py`
Subscribes to drone camera, processes RGB → occupancy grid.

Key methods:
- `compute_pixel_to_world_transform()` — affine mapping using altitude + FOV
- `process_image(rgb)` — grayscale → adaptive threshold → morphological cleanup → resize to 13×13 grid
- `pixel_to_world(px, py)` / `world_to_pixel(wx, wy)` — coordinate conversion

IoU metric vs ground-truth grid logged at end of PERCEIVING for thesis evaluation.

### 3. `maze_astar.py`
A* on the 13×13 expanded occupancy grid. Wall between adjacent cells detected at shared even-indexed coordinates.

Key methods:
- `plan(start_cell, end_cell)` — returns ordered list of `np.ndarray([x, y, z])` world waypoints
- `smooth_path(waypoints)` — line-of-sight pruning to reduce zigzagging

### 4. `maze_dog_controller.py`
Proportional waypoint follower outputting `Twist` commands directly (body frame).

```
max_linear_vel = 0.5 m/s
kp_linear = 1.0, kp_angular = 2.0
waypoint_radius = 0.3m (arrival threshold)
stuck_timeout = 10s
```

Key methods:
- `update(dog_pos, dog_yaw, dt)` → `(linear_x, angular_z)` — align heading first, then drive
- `is_done()`, `is_stuck(pos)`, `reset(waypoints)`

### 5. `maze_ros2_node.py`
Central ROS2 node. Follows `leader_follower_ros2_node.py` structure exactly (same timer loop, service client patterns, CSV logging).

**State machine:**
```
INIT → ARMING → CLIMBING → WAITING_HOVER → PERCEIVING → PLANNING → GUIDING_DOG
                                                              ↑           ↓
                                                          DOG_STUCK ←───┘
                                                              ↓
                                                     SUCCESS / FAILURE
```

Subscribers: `/drone/odom`, `/robot0/odom`, `/drone/front_cam/rgb` (via `cv_bridge`)
Publishers: `/robot0/cmd_vel`, `/maze/occupancy_grid` (OccupancyGrid), `/maze/planned_path` (Path)
Services: `/drone/arm`, `/drone/takeoff`

State notes:
- **WAITING_HOVER**: check `drone_z >= 10.5 * 0.95` + 5s minimum dwell
- **PERCEIVING**: accumulate 10 frames (10Hz = 1s), median-filter binary output, call `process_image`
- **PLANNING**: run A*; if no path, re-enter PERCEIVING (bad frame retry)
- **DOG_STUCK**: stop dog, re-enter PERCEIVING from current cell position
- **SUCCESS**: log mission summary (time, IoU, waypoints, collision count)

CSV columns: `t, state, dog_x, dog_y, drone_z, wp_idx, wp_total, vel_x, ang_z, dist_to_wp`

### 6. `run_maze.sh`
```
./run_maze.sh sim   # terminal 1 — Isaac Sim with maze
./run_maze.sh node  # terminal 2 — coordination node
```
Launches `main.py --robot go2 --robot_amount 1 --terrain flat --custom_env maze`.

---

## Existing Files to Modify (Minimal)

### `omniverse_sim.py` (~10 lines added)
In `setup_custom_env()`, add `elif custom_env == "maze":` branch:
- Import and call `MazeGenerator.spawn_walls_in_stage()`
- Set dog spawn position to maze start cell: `init_state.pos = (-2.5, -2.5, 0.5)`
- Call `create_world_drone_cam_omnigraph()` after world drone camera init (~line 926)

### `omnigraph.py` (~15 lines added)
Add `create_world_drone_cam_omnigraph()` — mirrors `create_front_cam_omnigraph()` but targets `/World/Drone/base/bottom_cam` and publishes to `/drone/front_cam/rgb`.

---

## Files Requiring Zero Changes
`drone_controller.py`, `apf_controller.py`, `ros2_bridge.py`, `leader_follower_ros2_node.py`, `custom_rl_env.py`, `lawnmower_planner.py`

---

## Implementation Order (Incremental Testing)

1. **`maze_generator.py`** — test standalone: print ASCII maze, verify wall count, test `get_occupancy_grid()`
2. **`maze_astar.py`** — test standalone using ground-truth grid from `MazeGenerator`; print path as ASCII overlay
3. **`maze_dog_controller.py`** — unit test with mock dog positions; check velocity convergence
4. **Sim integration** — add maze branch to `omniverse_sim.py`; run sim; confirm walls spawn and dog starts correctly
5. **`omnigraph.py`** — add world drone camera graph; confirm `/drone/front_cam/rgb` publishes at 10Hz
6. **`maze_vision.py`** — save one camera frame; tune threshold; confirm 13×13 grid matches ground truth
7. **`maze_ros2_node.py`** — wire everything; test state by state with debug `--start-state` arg
8. **End-to-end run** — full INIT→SUCCESS; collect CSV; compute IoU and mission time

---

## Success Criteria

| Metric | Target |
|---|---|
| Dog reaches end cell | Within 0.3m of (2.5, 2.5) |
| Mission time | ≤ 120s from CLIMBING start |
| Waypoint completion rate | ≥ 90% (wp_reached / wp_total) |
| Vision IoU vs ground truth | Log value (thesis baseline) |
| No out-of-bounds movement | Dog XY within [-3, 3] throughout |

---

## Critical File References

| File | Role |
|---|---|
| `omniverse_sim.py` | Add maze env branch + drone camera graph call |
| `omnigraph.py` | Add `create_world_drone_cam_omnigraph()` |
| `leader_follower_ros2_node.py` | Structural template for `maze_ros2_node.py` |
| `ros2_bridge.py` | Camera params (focal_length, aperture) for FOV/altitude math |
| `drone_controller.py` | POSITION mode PID behavior (0.5m tolerance) |
