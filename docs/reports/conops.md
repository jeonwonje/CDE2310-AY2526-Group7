# Concept of Operations (ConOps)

| Field          | Value                                              |
|----------------|----------------------------------------------------|
| Document ID    | AMR-CO-001                                         |
| Version        | 1.0                                                |
| Date           | 2026-04-13                                         |
| Author(s)      | Group 7 — Jeon, Kumaresan, Clara, Shashwat, Daniel |
| Module         | CDE2310 Engineering Systems Design                 |
| Status         | Baselined for G2                                   |

---

## 1  Purpose

This Concept of Operations describes the operational context, stakeholders,
mission phases, and high-level behaviour of the Group 7 Autonomous Mobile Robot.
It establishes the shared understanding between the development team and the
evaluation panel of *what* the system does, *for whom*, and *under what conditions*.

---

## 2  System Overview

The AMR is a TurtleBot3 Burger augmented with a custom spring-loaded ball launcher,
a 3-ball carousel magazine, and an RPi Camera V2. It autonomously explores a
previously unknown maze, identifies two delivery stations marked by AprilTag
fiducials, docks at each, and delivers ping-pong balls — all within a 25-minute
window and without human intervention.

---

## 3  Stakeholders

| Stakeholder           | Role                           | Concerns                                 |
|-----------------------|--------------------------------|------------------------------------------|
| NUS EDIC Panel        | Evaluator / client             | Mission success rate, design rigour, safety |
| Group 7 (dev team)    | Designers & operators          | Feasibility, integration risk, schedule  |
| TurtleBot3 platform   | Hardware provider (ROBOTIS)    | Payload limits, electrical compatibility |
| Future CDE2310 cohorts| Potential re-users             | Documentation clarity, reproducibility   |

---

## 4  System Boundary

```
 ┌──────────────────────────────── SYSTEM BOUNDARY ──────────────────────────────┐
 │                                                                               │
 │  ┌──────────── RPi 4B ────────────┐    Wi-Fi     ┌──── Laptop ────────────┐  │
 │  │  TurtleBot3 bringup            │◄────DDS────►│  Nav2 / Cartographer   │  │
 │  │  AprilTag detector (camera)     │   unicast    │  Mission coordinator   │  │
 │  │  RPi shooter node (GPIO/servo)  │              │  auto_explore_v2       │  │
 │  │  Delivery server                │              │  RViz visualisation    │  │
 │  └─────────────────────────────────┘              └────────────────────────┘  │
 │                                                                               │
 │  ┌─────────── Mechanical ──────────┐                                          │
 │  │  Spring-loaded launcher         │                                          │
 │  │  Carousel ball magazine (×3)    │                                          │
 │  │  3D-printed mounts & guides     │                                          │
 │  └─────────────────────────────────┘                                          │
 │                                                                               │
 └───────────────────────────────────────────────────────────────────────────────┘

 External:  Maze walls  |  AprilTag stations  |  Wi-Fi AP  |  Operator (start only)
```

---

## 5  Operational Phases

### 5.1  Phase 0 — Pre-Mission Setup

| Step | Action                                       | Actor    |
|------|----------------------------------------------|----------|
| 0.1  | Power on TurtleBot3, RPi, and laptop         | Operator |
| 0.2  | Launch RPi nodes (`turtlebot3_bringup`)      | Operator |
| 0.3  | Launch laptop nodes (Nav2, SLAM, coordinator)| Operator |
| 0.4  | Verify DDS discovery (`ros2 node list`)      | Operator |
| 0.5  | Place robot at maze entrance                 | Operator |
| 0.6  | Press start — mission timer begins           | Operator |

### 5.2  Phase 1 — Mission Initialisation

The `mission_coordinator_v3` node starts in `INIT` state. It waits for the
`toggle_exploration` service to become available, then transitions to `EXPLORING`.

### 5.3  Phase 2 — Autonomous Exploration

1. `find_frontiers` node performs BFS flood-fill on the `/map` occupancy grid
   to detect frontier cells (free cells adjacent to unknown cells).
2. Frontier cells are clustered by spatial proximity (minimum cluster size = 3).
3. `score_and_post` scores each cluster using BFS distance from the robot and
   cluster size. It optionally pre-flights the goal via `ComputePathToPose` to
   reject unreachable candidates.
4. The highest-scored goal is posted to Nav2 via `NavigateToPose` action.
5. On goal completion or failure, the cycle repeats.
6. When no frontiers remain, `score_and_post` publishes `EXPLORATION_COMPLETE`
   on `/mission_status`.

### 5.4  Phase 3 — Tag Detection (Concurrent)

Tag detection runs continuously in parallel with exploration:

1. `apriltag_detector` subscribes to `/camera/image_raw` on the RPi.
2. Each frame is converted to greyscale, and the `apriltag` library detects
   tag36h11 markers.
3. Detected tags are localised via `cv2.solvePnP` → 6-DOF pose in camera frame.
4. The pose is broadcast as a TF transform: `camera_link → tag36h11:<id>`.
5. The mission coordinator polls the TF tree at 10 Hz. If a target tag is found
   with age < 0.5 s, it interrupts exploration.

### 5.5  Phase 4 — Docking

On tag interrupt:

1. Coordinator stops exploration and sends `START_DOCKING` to the docking server.
2. Docking server computes a staging waypoint 0.60 m in front of the tag in the
   map frame and dispatches it to Nav2.
3. After Nav2 arrival, the server enters a 3-state visual-servoing loop:
   - **INTERCEPT**: Drive at a slant to reduce lateral (Y) error.
   - **SQUARE_UP**: Rotate in place to align yaw with the tag normal.
   - **FINAL_PLUNGE**: Drive straight until stop distance (0.10 m).
4. On success → `DOCKING_COMPLETE`. On failure → `DOCKING_FAILED` (tag is
   blacklisted for 30 s).

### 5.6  Phase 5 — Ball Delivery

1. Coordinator sends `START_DELIVERY` with the station tag name.
2. **Static station (tag36h11:0):** fire 3 balls in a timed sequence.
3. **Dynamic station (tag36h11:2):** wait for tag ID 3 to pass in front of the
   camera, fire on detection, respect 4 s cooldown, repeat up to 3 shots.
4. On completion → `DELIVERY_COMPLETE`.

### 5.7  Phase 6 — Search (Fallback)

If exploration completes but un-serviced tags remain:

1. Coordinator transitions to `SEARCHING`.
2. `search_stations` navigates to pre-computed search zones derived from the
   robot's starting position.
3. At each zone, the robot performs a full 360° spin (0.5 rad/s × 13 s) to scan
   for the missing tag.
4. If the tag is detected during spin → interrupt to docking. If all zones
   exhausted → mark tag as done (prevents infinite loops).

### 5.8  Phase 7 — Mission Complete / Shutdown

- Coordinator enters `MISSION_COMPLETE` when all target tags are in `docked_tags`.
- The operator observes via RViz and `/mission_status` topic.
- Manual shutdown: Ctrl-C on both machines.

---

## 6  Operational Scenarios

### 6.1  Nominal Scenario

```
 START
   │
   ▼
 EXPLORING ──tag seen──► DOCKING ──success──► DELIVERING ──done──► UNDOCKING
   ▲                        │                                         │
   │                        │ fail (blacklist 30 s)                   │
   │                        └───────────────────┐                     │
   │                                            ▼                     │
   └──────────────────── resume ◄──────────── EXPLORING               │
   │                                                                  │
   │◄─────────────────── resume ◄─────────────────────────────────────┘
   │
   ▼  (no frontiers left)
 SEARCHING ──tag seen──► DOCKING ──► ...
   │
   ▼  (all done)
 MISSION_COMPLETE
```

### 6.2  Degraded Scenario — Camera Dropout

If the camera feed drops during docking, the docking server coasts on the
last-known transform for up to 1.0 s (`sensor_drop_tolerance`). If the dropout
exceeds this threshold, the docking attempt aborts, the tag is blacklisted,
and exploration resumes.

### 6.3  Degraded Scenario — Nav2 Goal Rejection

If Nav2 rejects a staging waypoint (e.g., inside an obstacle), the docking server
subtracts the `fallback_staging_offset` (0.15 m) and retries. If that also fails,
docking is aborted.

---

## 7  Assumptions

| ID    | Assumption                                                              |
|-------|-------------------------------------------------------------------------|
| AS-01 | The maze has a single entrance and no dynamic obstacles.                |
| AS-02 | AprilTag markers are planar, rigidly mounted, and well-lit.             |
| AS-03 | Wi-Fi link between RPi and laptop is stable throughout the mission.     |
| AS-04 | The TurtleBot3 battery provides ≥ 25 minutes of continuous operation.   |
| AS-05 | Maze floor is flat and provides adequate traction for differential drive.|
| AS-06 | Manual time offset (~0.40 s RPi ahead) is acceptable without NTP sync. |

---

## 8  Risks

| ID    | Risk                                         | Likelihood | Impact | Mitigation                                |
|-------|----------------------------------------------|------------|--------|-------------------------------------------|
| RK-01 | Wi-Fi latency causes TF staleness            | Medium     | High   | Stale TF threshold (0.5 s), unicast DDS   |
| RK-02 | Docking failure from camera angle             | Medium     | Medium | 30 s blacklist + retry after re-exploration|
| RK-03 | Exploration misses a tag behind narrow gap    | Low        | High   | Search phase with pre-computed zones       |
| RK-04 | Launcher jam / misfire                        | Low        | Medium | 3-shot budget; no mechanical feedback loop |
| RK-05 | Battery depletion before mission end          | Low        | High   | LiDAR stop_distance prevents wall collisions|

---

## 9  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
