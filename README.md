# Group 7 — Autonomous Mobile Robot (AMR)

ROS 2 Humble workspace for the CDE2310 AMR project. The robot autonomously
explores an arena using LiDAR-based SLAM, detects AprilTag delivery stations,
docks via geometric visual servoing, and delivers ping-pong balls using a
servo-driven rack-and-pinion launcher.

## Architecture

```
                  ┌──────────────┐
                  │  Cartographer │  (SLAM)
                  │    + Nav2     │
                  └──────┬───────┘
                         │  /map, /odom, TF
         ┌───────────────┼───────────────┐
         v               v               v
┌────────────────┐ ┌───────────┐ ┌──────────────┐
│ auto_explore_v2│ │  Search   │ │   Docking    │
│ (frontiers)    │ │  Server   │ │   Server     │
└───────┬────────┘ └─────┬─────┘ └──────┬───────┘
        │                │              │
        └───────┬────────┘              │
                v                       v
        ┌───────────────┐       ┌──────────────┐
        │   Mission     │──────>│   Delivery   │
        │  Coordinator  │       │   Server     │
        │   (FSM)       │       └──────┬───────┘
        └───────────────┘              │ /fire_ball
                                       v
                                ┌──────────────┐
                                │ RPi Shooter  │  (GPIO servo)
                                └──────────────┘
```

**Mission flow:** Explore → Detect tag → Dock (0.6m staging → 0.1m final) →
Deliver (3 balls static / tag-triggered dynamic) → Undock → Resume.
Failed docks are blacklisted for 30s. After map exhaustion, a Search phase
sweeps predefined zones with 360-degree spins.

## Packages

| Package | Description |
|---|---|
| `CDE2310_AMR_Trial_Run` | Mission coordinator FSM, search, docking, delivery, launcher nodes, Nav2 + Cartographer configs |
| `auto_explore_v2` | BFS frontier detection (`find_frontiers`) and scored goal posting (`score_and_post`) |

## Repository Layout

```
Group7_AMR/                 ← colcon workspace root
├── src/
│   ├── CDE2310_AMR_Trial_Run/
│   │   ├── CDE2310_AMR_Trial_Run/   ← Python nodes
│   │   ├── launch/                   ← Launch files
│   │   └── config/                   ← Nav2 YAML, Cartographer Lua
│   └── auto_explore_v2/
│       ├── auto_explore_v2/          ← Frontier exploration nodes
│       ├── launch/
│       └── config/
├── hardware/
│   ├── chassis/            ← TurtleBot3 assembly CAD (SolidWorks, 3MF)
│   └── launcher/           ← Rack-and-pinion launcher CAD (v1.0, v1.1)
├── docs/
│   ├── reports/            ← Systems engineering reports (ConOps, HLD, ICD, etc.)
│   └── end_user_doc/       ← End-user documentation
├── data/                   ← Recorded bags, maps
└── archive/                ← Legacy lab exercise nodes (frozen)
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Group7_AMR
colcon build
source install/setup.bash
```

Incremental build for a single package:

```bash
colcon build --packages-select CDE2310_AMR_Trial_Run
```

## Launch

### Gazebo Simulation

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py
```

Nav-only (no mission/exploration nodes):

```bash
ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py launch_mission:=false
```

### Real Robot (Remote PC)

Single command (replaces 4 terminals):

```bash
ros2 launch CDE2310_AMR_Trial_Run full_mission.launch.py
```

Or launch components individually:

| Terminal | Command |
|---|---|
| T1 — Bringup | `export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_bringup robot.launch.py` |
| T2 — Nav2 + SLAM | `ros2 launch CDE2310_AMR_Trial_Run minimal_nav2.launch.py use_sim_time:=false` |
| T3 — Exploration | `ros2 launch auto_explore_v2 auto_explore.launch.py use_sim_time:=false` |
| T4 — Mission | `ros2 launch CDE2310_AMR_Trial_Run mission.launch.py use_sim_time:=false` |
| T5 — Shooter (RPi) | `ros2 run CDE2310_AMR_Trial_Run rpi_shooter_node` |

### Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `false` (`true` for Gazebo) | Use simulation clock |
| `launch_mission` | `true` | Enable mission nodes (Gazebo launch only) |

## Nodes

### CDE2310_AMR_Trial_Run

| Node | Entry Point | Role |
|---|---|---|
| `mission_coordinator` | `mission_coordinator_v3.py` | Central FSM: EXPLORING → DOCKING → DELIVERING → UNDOCKING |
| `search_server` | `search_stations.py` | Post-exploration zone sweeps with 360-degree scans |
| `docking_server` | `docker.py` | Geometric visual servoing to AprilTag stations |
| `delivery_server` | `delivery_server.py` | Static (timed) and dynamic (tag-triggered) ball delivery |
| `rpi_shooter_node` | `rpi_shooter_node.py` | RPi GPIO servo control, `/fire_ball` service |
| `launcher_node` | `launcher_node.py` | Mock shooter for testing without hardware |
| `apriltag_detector` | `apriltag_detector.py` | Backup tag ID 3 monitor for dynamic station |

### auto_explore_v2

| Node | Entry Point | Role |
|---|---|---|
| `find_frontiers` | `find_frontiers.py` | BFS frontier detection on occupancy grid |
| `score_and_post` | `score_and_post.py` | Frontier scoring (distance + blacklist) and Nav2 goal posting |

## Hardware

- **Platform:** TurtleBot3 Waffle Pi (LDS-02 LiDAR, OpenCR 1.0)
- **Camera:** Raspberry Pi Camera Module V2 (CSI)
- **Launcher:** MG90 continuous-rotation servo, rack-and-pinion plunger (GPIO 12, 50Hz PWM)
- **Detection:** AprilTag 36h11 family (tag IDs 0, 2, 3)
- **Capacity:** 3 standard 40mm ping-pong balls

## Dependencies

- ROS 2 Humble
- `cartographer_ros`
- `nav2_bringup` (Nav2 stack)
- `turtlebot3_gazebo` (simulation only)
- Python: `py-trees`, `apriltag`, `pyserial`

## Team

| Member | Role | Branch |
|---|---|---|
| Jeon Won Je | Systems lead, manufacturing | `dev/jeon` |
| Clara | Navigation, launcher | `dev/clara` |
| Kumaresan | Navigation | `dev/kumaresan` |
| Shashwat | Perception | `dev/shashwat` |
| Daniel | Mechanical subsystems | `dev/daniel` |
