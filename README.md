# Group 7 — Autonomous Mobile Robot (AMR)

> CDE2310 Engineering Systems Design · AY25/26 Semester 2 · NUS EDIC

ROS 2 Humble workspace for the CDE2310 warehouse delivery mission. The robot autonomously explores a maze using LiDAR-based SLAM, detects AprilTag delivery stations, docks via geometric visual servoing, and delivers ping-pong balls using a servo-driven rack-and-pinion launcher.

---

## Table of Contents

1. [Concept of Operations](#1-concept-of-operations)
2. [Requirements](#2-requirements)
3. [High-Level Design](#3-high-level-design)
4. [Subsystem Design](#4-subsystem-design)
5. [Interface Control](#5-interface-control)
6. [Software Development](#6-software-development)
7. [Testing](#7-testing)
8. [End User Documentation](#8-end-user-documentation)
9. [Team](#9-team)

---

## 1. Concept of Operations

### Mission Overview

The AMR performs a fully autonomous warehouse delivery mission within a 25-minute window. It navigates an unknown maze, locates two delivery stations via AprilTag fiducial markers, and delivers 3 ping-pong balls at each station.

### System Overview

A spring-loaded rack-and-pinion launcher mounted on a TurtleBot3 Waffle Pi. An MG90 continuous-rotation servo compresses and releases a spring, propelling balls through a 42.5 mm barrel with deterministic force (1–2 m/s exit velocity). A 3-ball gravity-fed magazine feeds the barrel.

### Mission Flow

```
EXPLORE ──tag seen──► DOCK ──success──► DELIVER ──done──► UNDOCK ──► RESUME
    ▲                   │                                              │
    │                   └── fail (blacklist 30s) ──────────────────────┘
    │
    ▼ (map exhausted, tags remain)
 SEARCH ──tag seen──► DOCK ──► ...
    │
    ▼ (all stations serviced)
 MISSION COMPLETE
```

### Operational Scenarios

**Station A (static, tag 0):** Dock → fire → 4 s wait → fire → 6 s wait → fire. Total ~12 s.

**Station B (dynamic, tag 2):** Dock → subscribe to `/detections` → fire on tag ID 3 detection → 4 s cooldown → repeat up to 3 shots.

**Failure recovery:** Failed docks are blacklisted for 30 s. Camera dropouts tolerated for 1 s. Nav2 rejections retried with 0.15 m staging offset. If exploration completes with unserviced tags, a zone-sweep search phase begins.

### Design Selection

Three launcher concepts were evaluated via a weighted decision matrix (determinism 25%, manufacturability 20%, simplicity 15%, controllability 15%, power 10%, cost 10%, safety 5%):

| Concept | Score |
|---|---|
| **A — Spring plunger (rack-and-pinion servo)** | **4.45** |
| B — Dual flywheel (counter-rotating DC motors) | 2.90 |
| C — Pneumatic piston (solenoid + air reservoir) | 2.50 |

Concept A was selected for its deterministic energy transfer (½kx²), mechanical simplicity, and FDM manufacturability.

### Risks

| ID | Risk | Status |
|---|---|---|
| R1 | Barrel/plunger binding (FDM tolerance) | Resolved — 0.3 mm compensation + sanding |
| R2 | Spring fatigue over repeated cycles | Monitored — no degradation observed |
| R3 | Unreliable AprilTag vision for Station B | Resolved — vision validated; open-loop fallback available |

---

## 2. Requirements

### Functional Requirements

| ID | Requirement | Traces to |
|---|---|---|
| FR-EXP-01 | Build occupancy grid map via Cartographer SLAM | ConOps §5.3 |
| FR-EXP-02 | Detect frontier cells via BFS flood-fill | ConOps §5.3 |
| FR-EXP-03 | Score frontiers by BFS distance and cluster size | ConOps §5.3 |
| FR-EXP-04 | Navigate to highest-scored frontier until none remain | ConOps §5.3 |
| FR-DET-01 | Detect tag36h11 markers using RPi Camera V2 | ConOps §5.4 |
| FR-DET-02 | Compute 6-DOF pose via solvePnP and publish as TF | ConOps §5.4 |
| FR-DCK-01 | Navigate to staging point 0.60 m from tag | ConOps §5.5 |
| FR-DCK-02 | Execute discrete geometric visual servoing (intercept → square-up → plunge) to dock within 0.10 m | ConOps §5.5 |
| FR-DCK-03 | Blacklist tag for 30 s on dock failure | ConOps §5.5 |
| FR-DEL-01 | Fire 3 balls at static station (4 s, 6 s gaps) | ConOps §5.6 |
| FR-DEL-02 | Fire reactively at dynamic station on tag ID 3 detection with 4 s cooldown | ConOps §5.6 |
| FR-MSN-01 | Central FSM orchestrates all phases | ConOps §5.2 |
| FR-MSN-02 | Transition to SEARCHING when exploration complete but tags remain | ConOps §5.7 |

### Non-Functional Requirements

| ID | Category | Requirement |
|---|---|---|
| NFR-01 | Timing | Full mission ≤ 25 minutes |
| NFR-02 | Timing | Tag detection latency ≤ 100 ms at 10 Hz |
| NFR-03 | Accuracy | Docking lateral error ≤ 0.02 m |
| NFR-04 | Accuracy | Docking yaw error ≤ 0.05 rad (≈ 3°) |
| NFR-05 | Reliability | Tolerate camera dropout for up to 1.0 s |
| NFR-06 | Reliability | Handle Nav2 goal rejection with fallback staging offset |
| NFR-07 | Power | Operate from TurtleBot3 12 V battery for full mission |
| NFR-08 | Comms | FastDDS unicast (no multicast) between RPi and laptop |

### Constraints

| ID | Constraint |
|---|---|
| CON-01 | 25-minute mission window |
| CON-02 | Platform: TurtleBot3 Waffle Pi, no base modification |
| CON-03 | Payload: exactly 3 ping-pong balls |
| CON-04 | Software: ROS 2 Humble on Ubuntu 22.04 |
| CON-05 | No manual intervention after mission start |

---

## 3. High-Level Design

### Architecture

Two-machine distributed ROS 2 system. Compute-heavy navigation and planning run on a laptop; hardware-coupled perception and actuation run on the Raspberry Pi.

```
┌───────────────────────────────────────────────────────────────────────────┐
│                        ROS 2 Humble (FastDDS)                             │
│                                                                           │
│  ┌──────────── Laptop ───────────────┐  ┌────────── RPi 4B ───────────┐  │
│  │                                   │  │                             │  │
│  │  Cartographer SLAM                │  │  turtlebot3_bringup         │  │
│  │         │                         │  │  (OpenCR, LDS-02)           │  │
│  │         ▼                         │  │                             │  │
│  │  Nav2 (planner, controller)       │  │  apriltag_detector          │  │
│  │         │                         │  │  /camera → TF, /marker      │  │
│  │         ▼                         │  │                             │  │
│  │  auto_explore_v2                  │  │  delivery_server            │  │
│  │  (frontiers → goals)              │  │  (shot orchestration)       │  │
│  │         │                         │  │                             │  │
│  │         ▼                         │  │  rpi_shooter_node           │  │
│  │  mission_coordinator (FSM)        │  │  (GPIO/servo PWM)           │  │
│  │                                   │  │                             │  │
│  │  docking_server                   │  │                             │  │
│  │  search_stations                  │  │                             │  │
│  └───────────────────────────────────┘  └─────────────────────────────┘  │
│                                                                           │
│          ◄──────── FastDDS unicast over Wi-Fi ────────►                   │
└───────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
 LDS-02 LiDAR         RPi Camera V2
      │                     │
      ▼                     ▼
 Cartographer         apriltag_detector
      │                     │
      ├──► /map             ├──► TF: camera_link → tag36h11:<id>
      ▼                     ▼
 find_frontiers       mission_coordinator (TF poll)
      │                     │
      ▼                     ▼
 score_and_post       docking_server → delivery_server → rpi_shooter_node → Ball
      │
      ▼
 Nav2 → /cmd_vel → OpenCR
```

### Packages

| Package | Description | Machine |
|---|---|---|
| `auto_explore_v2` | BFS frontier detection, scored Nav2 goals | Laptop |
| `CDE2310_AMR_Trial_Run` | Central FSM, docking, delivery, search, launcher | Laptop + RPi |

### Hardware-Software Mapping

```
┌──────────────────────────────────────────────────────────────┐
│                     TurtleBot3 Waffle Pi                      │
│                                                               │
│  Layer 4 (top): RPi Camera V2 → RPi 4B → OpenCR             │
│                 Launcher servo ← GPIO/PWM ← RPi 4B          │
│                                                               │
│  Layer 3: Launcher assembly (spring plunger, barrel, gear)   │
│           3D-printed launcher mount v2                         │
│                                                               │
│  Layer 2: OpenCR → Dynamixel XL430 (L/R wheels)             │
│           LDS-02 LiDAR (360° @ 5 Hz)                         │
│                                                               │
│  Layer 1: LiPo 12V → Buck 5V → RPi                          │
└──────────────────────────────────────────────────────────────┘
         ◄── Wi-Fi ──►  Laptop (Nav2, SLAM, Coordinator)
```

### Design Decisions

| ID | Decision | Rationale |
|---|---|---|
| DD-01 | Two-machine split (RPi + laptop) | RPi lacks compute for Nav2 + SLAM; laptop cannot access GPIO |
| DD-02 | JSON-encoded String topics for command/status | Avoids custom msg definitions; fast iteration |
| DD-03 | Discrete geometric docking (not PID) | Eliminates gain-tuning; state machine more debuggable |
| DD-04 | BFS frontier detection (not RRT) | Simpler to implement and debug; sufficient for maze |
| DD-05 | Tag blacklisting on dock failure | Prevents infinite retry loops |

---

## 4. Subsystem Design

### 4.1 Navigation — `auto_explore_v2`

**Owner:** Kumaresan

**Frontier detection** (`find_frontiers.py`): Subscribes to `/map`, builds cell dictionary, identifies free cells with unknown neighbours via 4-connected BFS, clusters contiguous frontier cells (min size 3), publishes centroids and BFS distance transform.

**Frontier scoring** (`score_and_post.py`): Scores clusters by size and BFS distance, pre-flights top candidate via `ComputePathToPose`, posts winning goal to Nav2, publishes `EXPLORATION_COMPLETE` when no clusters remain.

| Parameter | Value | Description |
|---|---|---|
| FRONTIER_MIN_SIZE | 3 cells | Minimum cluster size |
| PATH_BLOCKED_OCC_MIN | 51 | Occupancy threshold for blocked path |
| PREFLIGHT_TIMEOUT_SEC | 10.0 s | ComputePathToPose timeout |

### 4.2 Perception — `apriltag_detector`

**Owner:** Clara

Subscribes to `/camera/image_raw`, runs `apriltag.Detector(families='tag36h11')` at 10 Hz, computes 6-DOF pose via `cv2.solvePnP`, broadcasts TF transforms `camera_link → tag36h11:<id>`.

| Parameter | Value | Description |
|---|---|---|
| marker_size | 0.16 m | Tag physical side length |
| detection_rate | 10.0 Hz | Processing throttle |

### 4.3 Docking — `docker.py`

**Owner:** Shashwat

Three-phase discrete geometric visual servoing:

```
Nav2 staging (0.60 m) → INTERCEPT (reduce Y error to ≤ 0.02 m)
                       → SQUARE_UP (align yaw to ≤ 0.05 rad)
                       → FINAL_PLUNGE (drive to 0.10 m)
```

Robustness: camera dropout coast (1 s), Nav2 rejection retry with −0.15 m offset, backup-and-retry on excessive Y error, 180 s hard timeout.

### 4.4 Delivery

**Owner:** Clara (launcher/shooter nodes), Jeon (delivery_server)

`delivery_server`: Orchestrates static delivery (timed 3-shot sequence) and dynamic delivery (reactive firing on tag ID 3 detection with 4 s cooldown).

`rpi_shooter_node`: Provides `/fire_ball` service on RPi. Activates MG90 servo via GPIO 12 (50 Hz PWM, duty 10.0 CCW) for one fire cycle (0.87 s).

### 4.5 Mission Coordination — `mission_coordinator_v3`

**Owner:** Kumaresan, Jeon (robustness patches)

Central FSM: INIT → EXPLORING → DOCKING → DELIVERING → UNDOCKING → (resume or SEARCHING → MISSION_COMPLETE).

| Mechanism | Implementation |
|---|---|
| Tag monitoring | Poll TF tree at 10 Hz |
| Staleness filtering | Reject transforms older than 0.5 s |
| Blacklisting | HashMap `{tag: expiry_time}`, 30 s |
| Exploration toggle | `toggle_exploration` SetBool service |
| Command dispatch | JSON on `/mission_command` |

### 4.6 Search — `search_stations`

**Owner:** Kumaresan

When exploration is complete but tags remain, navigates to pre-computed zones, performs 360° scans (0.5 rad/s × 13 s). Aborts search on tag detection.

---

## 5. Interface Control

### ROS 2 Topics

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/map` | OccupancyGrid | Cartographer | find_frontiers, search_stations |
| `/scan` | LaserScan | LDS-02 driver | Cartographer |
| `/cmd_vel` | Twist | Nav2 / docking / search | OpenCR |
| `/camera/image_raw` | Image | RPi camera | apriltag_detector |
| `/mission_command` | String (JSON) | mission_coordinator | docker, delivery, search |
| `/mission_status` | String (JSON) | docker, delivery, search, explorer | mission_coordinator |
| `/detections` | AprilTagDetectionArray | apriltag (RPi) | delivery_server |
| `/goal_pose` | PoseStamped | score_and_post | Nav2 |
| `frontiers` | String (JSON) | find_frontiers | score_and_post |

### ROS 2 Services

| Service | Type | Server | Client |
|---|---|---|---|
| `toggle_exploration` | SetBool | score_and_post | mission_coordinator |
| `/fire_ball` | Trigger | rpi_shooter_node | delivery_server |

### ROS 2 Actions

| Action | Type | Server | Client(s) |
|---|---|---|---|
| `navigate_to_pose` | NavigateToPose | Nav2 | score_and_post, docker, search |
| `compute_path_to_pose` | ComputePathToPose | Nav2 | score_and_post |

### Mission Command Protocol

```json
Command: {"action": "START_DOCKING", "target": "tag36h11:0"}
Status:  {"sender": "docker", "status": "DOCKING_COMPLETE", "data": "tag36h11:0"}
```

| Command | Consumer | Status Response |
|---|---|---|
| `START_DOCKING` | docker | `DOCKING_COMPLETE` / `DOCKING_FAILED` |
| `START_UNDOCKING` | docker | `UNDOCKING_COMPLETE` |
| `START_DELIVERY` | delivery_server | `DELIVERY_COMPLETE` |
| `START_SEARCH` | search_stations | (tag interrupt or `SEARCH_FAILED`) |

### TF Tree

```
map → odom → base_footprint → base_link → base_scan (LiDAR)
                                         → camera_link → tag36h11:0 (dynamic)
                                                       → tag36h11:2 (dynamic)
                                                       → tag36h11:3 (dynamic)
```

### Hardware Interfaces

| Interface | Pin/Protocol | Connected To |
|---|---|---|
| GPIO 18 (PWM) | 50 Hz PWM | Launcher servo |
| GPIO 23 | Digital out | Carousel motor |
| USB serial | 115200 baud | RPi ↔ OpenCR |
| CSI (MIPI) | Camera V2 | RPi CSI port |

### Network

| Parameter | Value |
|---|---|
| DDS | FastDDS, unicast (no multicast) |
| ROS_DOMAIN_ID | 30 |
| Time sync | Manual ~0.40 s offset; stale TF threshold absorbs drift |

---

## 6. Software Development

### Environment

| Component | Detail |
|---|---|
| OS | Ubuntu 22.04 (laptop + RPi) |
| ROS | Humble Hawksbill |
| Python | 3.10 |
| Build | colcon + ament_python |
| DDS | FastDDS via `rmw_fastrtps_cpp` |
| Linter | flake8, pep257 |

### Repository Layout

```
Group7_AMR/
├── src/
│   ├── auto_explore_v2/           # Frontier exploration
│   │   ├── auto_explore_v2/       #   find_frontiers.py, score_and_post.py
│   │   ├── config/                #   nav2_params.yaml
│   │   └── launch/                #   auto_explore.launch.py
│   └── CDE2310_AMR_Trial_Run/     # Mission coordination
│       ├── CDE2310_AMR_Trial_Run/ #   coordinator, docker, delivery, search, shooter
│       ├── config/                #   slam_params.yaml, minimal_nav2.yaml
│       └── launch/                #   mission, full_mission, minimal_nav2, gazebo
├── hardware/
│   ├── chassis/                   # TurtleBot3 assembly + mounts (CAD)
│   └── launcher/                  # Launcher mechanism (CAD + 3MF)
├── docs/
│   ├── reports/                   # Full SDD reports (detailed versions)
│   └── end_user_doc/              # Printed end-user documentation
├── data/                          # Maps, bag files
├── CHANGELOG.md
└── README.md                      # ← You are here
```

### Branch Policy

```
main                    ← stable, merged via PR only
├── dev/jeon            ← Systems lead, manufacturing, delivery
├── dev/clara           ← Navigation, launcher, perception
├── dev/kumaresan       ← Navigation, coordination
├── dev/shashwat        ← Docking, perception
└── dev/daniel          ← Mechanical subsystems
```

**Rules:** No direct pushes to `main`. All changes via PR. Build must pass. At least 1 review. Squash merge preferred.

### Commit Conventions

[Conventional Commits](https://www.conventionalcommits.org/):

```
feat(dock): add fallback staging offset on Nav2 rejection
fix(delivery): prevent double-fire during cooldown window
docs(report): add G2 systems design documentation
```

Types: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`
Scopes: `nav`, `explore`, `dock`, `delivery`, `perception`, `launcher`, `mission`, `hardware`, `report`

### AI Attribution

AI assistants (Claude, GitHub Copilot) are used for code and documentation. All AI contributions are attributed via commit trailers: `Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>` and `ai-assisted: yes`. The team reviews and approves all AI-generated content.

### Versioning

[Semantic Versioning](https://semver.org/) — `MAJOR.MINOR.PATCH`. All changes recorded in [CHANGELOG.md](CHANGELOG.md).

### Build & Launch

```bash
# Build
source /opt/ros/humble/setup.bash
cd ~/Group7_AMR && colcon build && source install/setup.bash

# Gazebo simulation
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py

# Real robot (single command)
ros2 launch CDE2310_AMR_Trial_Run full_mission.launch.py

# Or launch individually:
# T1: ros2 launch turtlebot3_bringup robot.launch.py
# T2: ros2 launch CDE2310_AMR_Trial_Run minimal_nav2.launch.py use_sim_time:=false
# T3: ros2 launch auto_explore_v2 auto_explore.launch.py use_sim_time:=false
# T4: ros2 launch CDE2310_AMR_Trial_Run mission.launch.py use_sim_time:=false
# T5: ros2 run CDE2310_AMR_Trial_Run rpi_shooter_node  (on RPi)
```

### Dependencies

**ROS 2:** rclpy, std_msgs, std_srvs, geometry_msgs, nav_msgs, sensor_msgs, nav2_msgs, tf2_ros, cv_bridge, cartographer_ros, nav2_bringup, turtlebot3_bringup

**Python:** numpy, opencv-python, apriltag, pytest

---

## 7. Testing

### Strategy

| Level | Scope | Tools | Automated? |
|---|---|---|---|
| Unit | Functions / algorithms | pytest | Yes |
| Lint | PEP 8, docstrings | flake8, pep257 | Yes |
| Integration | Node-to-node comms | Manual + ros2 CLI | Partial |
| System (FAT) | Full mission end-to-end | Physical maze run | No |

### Unit & Lint Tests

Each package includes ament standard lint tests (flake8, pep257, copyright):

```bash
colcon test
colcon test-result --verbose
```

### Integration Tests

| ID | Test | Verifies |
|---|---|---|
| TST-INT-01 | Exploration publishes frontiers | FR-EXP-02 |
| TST-INT-02 | AprilTag TF broadcast | FR-DET-02 |
| TST-INT-03 | toggle_exploration service | FR-MSN-01 |
| TST-INT-04 | /fire_ball triggers servo | FR-DEL-03 |
| TST-INT-05 | mission_command → docker | FR-DCK-01 |
| TST-INT-06 | DDS cross-machine topic flow | NFR-08 |

### Factory Acceptance Test (FAT)

| ID | Criterion | Pass Condition |
|---|---|---|
| TST-SYS-01 | Full maze exploration | ≥ 90% coverage within 15 min |
| TST-SYS-02 | Tag detection | Both station tags detected |
| TST-SYS-03 | Docking (Station A) | Within 0.10 m, Y error ≤ 0.02 m |
| TST-SYS-04 | Static delivery | 3 balls fired |
| TST-SYS-05 | Docking (Station B) | Same as TST-SYS-03 |
| TST-SYS-06 | Dynamic delivery | ≥ 1 ball on tag detection |
| TST-SYS-07 | Mission completion | Coordinator reaches MISSION_COMPLETE |
| TST-SYS-08 | Total time | ≤ 25 min |
| TST-SYS-09 | No manual intervention | Fully autonomous |

### Requirement Traceability

| Req ID | Design § | Test § |
|---|---|---|
| FR-EXP-01–05 | §4.1 Navigation | TST-INT-01, TST-SYS-01 |
| FR-DET-01–02 | §4.2 Perception | TST-INT-02, TST-SYS-02 |
| FR-DCK-01–03 | §4.3 Docking | TST-INT-05, TST-SYS-03/05 |
| FR-DEL-01–02 | §4.4 Delivery | TST-INT-04, TST-SYS-04/06 |
| FR-MSN-01–02 | §4.5 Coordination | TST-INT-03, TST-SYS-07 |

### Known Issues

| # | Issue | Severity | Status |
|---|---|---|---|
| #5 | Camera flicker causes false tag detections | Medium | Mitigated (stale TF threshold) |
| #6 | Nav2 goal rejection near map boundaries | Medium | Mitigated (fallback offset) |
| #7 | Time offset drift between RPi and laptop | Low | Open |
| #8 | Carousel indexing unreliable after 3rd ball | Low | Open (not mission-critical) |

---

## 8. End User Documentation

### Specifications

| Spec | Value |
|---|---|
| Dimensions | 281 × 306 × ~300 mm (with payload) |
| Platform | TurtleBot3 Waffle Pi |
| Power | LB-012, 11.1 V 1800 mAh LiPo (~80 min normal) |
| LiDAR | LDS-02, 360°, 0.12–3.5 m |
| Camera | RPi Camera Module V2 (CSI) |
| Launcher | MG90 servo, rack-and-pinion, GPIO 12, 50 Hz PWM |
| Ball capacity | 3 × 40 mm ping-pong balls |
| Launch cycle | 0.87 s per ball |

### Quick Start

1. Load 3 balls into magazine
2. Power on OpenCR, wait ~30 s for RPi boot
3. SSH into RPi: `ssh ubuntu@<ROBOT_IP>`
4. Launch: `ros2 launch CDE2310_AMR_Trial_Run full_mission.launch.py`
5. Robot explores, detects, docks, delivers autonomously
6. Shutdown: `Ctrl+C` all sessions, switch off OpenCR

### Troubleshooting

| Issue | Resolution |
|---|---|
| Robot doesn't move | Check OpenCR power; reseat USB RPi ↔ OpenCR |
| No frontiers found | Map fully explored or SLAM not running |
| AprilTag not detected | Check camera CSI cable; verify apriltag node running |
| Docking aborts | Check lighting; increase blacklist_timeout |
| Launcher jams | Clear barrel; check gear alignment; sand barrel interior |
| Servo not firing | Verify GPIO 12 connection; test with `raspi-gpio` |

### Safety

- Keep fingers clear of launcher barrel and mechanism during operation
- Do not discharge LiPo below 9 V. Store in fireproof bag
- Robot moves without warning — maintain clear perimeter
- **Emergency stop:** Press OpenCR reset button

---

## 9. Team

| Member | Role | Branch |
|---|---|---|
| Jeon Won Je | Systems lead, delivery server, manufacturing | `dev/jeon` |
| Clara Ong | Navigation, launcher, perception | `dev/clara` |
| Kumaresan | Navigation, mission coordination | `dev/kumaresan` |
| Shashwat Gupta | Docking, launcher mechanism | `dev/shashwat` |
| Daniel Yow | Mechanical subsystems, CAD | `dev/daniel` |

---

*CDE2310 · NUS College of Design and Engineering · AY25/26*
