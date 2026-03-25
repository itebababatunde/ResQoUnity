# Search & Rescue Thesis — Aerial-Assisted Maze Navigation

## Goal
Verify that a drone can provide aerial support for a quadruped robot (Go2) to navigate through a maze — using the drone's bird's-eye view to guide the dog when needed.

## System Participants
- 1 drone (aerial observer/guide) — downward facing camera
- 1 quadruped robot (Go2) — ground navigator

## Drone Capabilities Needed
- Is aware of dog position
- Can see the full maze from above
- Computes shortest valid path from start to finish
- Publishes navigation commands to guide the dog robot through the maze

---

## Assessment Questions for Claude Code

### 1. Existing APF Implementation
- What does the current APF leader-follower implementation look like — is the drone the leader or the dog?
- Is the APF purely for collision avoidance, formation keeping, or actual goal-directed navigation?
- Does the APF implementation already handle dynamic replanning, or is it a fixed potential field?

### 2. Perception & Mapping
- Is the downward facing camera mounted and publishing a feed in simulation?
- Can the drone fly high enough to see the full maze in a single frame?
- Do we have a maze representation (occupancy grid, USD, graph) that the path planner can consume?
- Is there a mechanism to convert the camera feed into a navigable map, or is the map given a priori?

### 3. Path Planning
- Do we have a shortest path finding algorithm implemented (A*, Dijkstra, RRT)?
- Is the planner integrated with the drone's camera perception, or does it assume a known map?
- Can the planner replan in real time if the dog deviates or gets stuck?

### 4. Drone–Dog Communication
- How exactly do the drone and dog communicate — shared ROS2 topics, services, or actions?
- What is the message format for navigation commands sent from drone to dog?
- Is there latency or sync issues between the drone's aerial view and the dog's ground execution?

### 5. Dog Robot Navigation
- How does the dog receive and execute navigation commands — velocity commands, waypoints, or high-level goals?
- Does the dog have its own local obstacle avoidance, or does it fully rely on the drone?
- Is the Go2's locomotion controller already integrated in the simulation?

### 6. Simulation Environment
- Do we have a maze USD asset ready for Isaac Sim?
- Is the maze scale appropriate for the Go2's physical dimensions?
- Are both the drone and Go2 spawned and stable in the same simulation scene?

### 7. Success Criteria
- What defines a successful run — dog reaches end of maze, within a time limit, without collisions?
- How do we log and evaluate performance across runs?
