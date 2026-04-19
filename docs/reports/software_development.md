# Software Development Plan

| Field          | Value                                              |
|----------------|----------------------------------------------------|
| Document ID    | AMR-SDP-001                                        |
| Version        | 1.0                                                |
| Date           | 2026-04-13                                         |
| Author(s)      | Group 7 — Jeon, Shashwat, Kuga, Clara, Daniel |
| Module         | CDE2310 Engineering Systems Design                 |
| Status         | Baselined for G2                                   |

---

## 1  Purpose

This document specifies the development environment, build toolchain, repository
structure, branching strategy, coding standards, and dependency management for
the Group 7 AMR project.

---

## 2  Development Environment

| Component      | Version / Detail                                   |
|----------------|----------------------------------------------------|
| OS (laptop)    | Ubuntu 22.04 LTS (Jammy Jellyfish)                |
| OS (RPi)       | Ubuntu 22.04 Server (arm64)                        |
| ROS distro     | ROS 2 Humble Hawksbill                             |
| Python         | 3.10.x (system Python, no virtual env for ROS)     |
| DDS middleware  | CycloneDDS (`rmw_cyclonedds_cpp`); FastRTPS (`rmw_fastrtps_cpp`) only for Gazebo on WSL2 |
| Build system   | colcon (v0.15+)                                    |
| Package format | ament_python                                       |
| IDE            | VS Code with ROS extension + Python extension      |
| Linter         | flake8, pep257                                     |
| Formatter      | (not enforced project-wide; flake8 for compliance) |

### 2.1  ROS 2 Workspace Setup

```bash
# Clone
mkdir -p ~/amr_ws/src && cd ~/amr_ws/src
git clone git@github.com:<org>/Group7_AMR.git .

# Install dependencies
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### 2.2  RPi Cross-Compilation (Not Used)

All RPi packages are built natively on the RPi. Cross-compilation was evaluated
and rejected due to complexity with camera and GPIO dependencies.

---

## 3  Repository Structure

```
Group7_AMR/
├── src/
│   ├── auto_explore_v2/           # Frontier exploration (Nav2)
│   │   ├── auto_explore_v2/
│   │   │   ├── __init__.py
│   │   │   ├── find_frontiers.py
│   │   │   └── score_and_post.py
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── launch/
│   │   │   └── auto_explore.launch.py
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── CDE2310_AMR_Trial_Run/     # Mission coordination
│   │   ├── CDE2310_AMR_Trial_Run/
│   │   │   ├── __init__.py
│   │   │   ├── mission_coordinator_v3.py
│   │   │   ├── docker.py
│   │   │   ├── delivery_server_consolidated.py
│   │   │   └── search_stations.py
│   │   ├── config/
│   │   │   ├── slam_params.yaml
│   │   │   └── minimal_nav2.yaml
│   │   ├── launch/
│   │   │   ├── mission.launch.py
│   │   │   ├── full_mission.launch.py
│   │   │   └── minimal_nav2.launch.py
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── apriltag_docking/          # C++ perception pipeline on RPi
│       ├── src/
│       │   └── detected_dock_pose_publisher.cpp
│       ├── include/
│       ├── launch/
│       │   └── apriltag_dock_pose_publisher.launch.py
│       ├── config/
│       │   └── apriltags_36h11.yaml
│       ├── CMakeLists.txt
│       └── package.xml
│
├── hardware/
│   ├── chassis/                   # TurtleBot3 assembly + mounts
│   │   ├── assembly/
│   │   ├── mounts/
│   │   └── 3mf/
│   └── launcher/                  # Launcher mechanism
│       ├── components/
│       ├── 3mf/
│       └── v1/
│
├── docs/
│   ├── reports/                   # Systems engineering documents
│   └── guides/                    # Developer guides
│
├── data/                          # Maps, bag files, logs
├── CHANGELOG.md
├── CLAUDE.md
├── AGENT_GIT_GUIDE.md
└── README.md
```

---

## 4  Branch Policy & Git Workflow

### 4.1  Branch Structure

```
main                      ← stable releases only
  │
  ├── dev/jeon            ← Jeon's development branch (lead)
  ├── dev/shashwat        ← Shashwat's development branch
  ├── dev/kuga            ← Kuga's development branch
  ├── dev/clara           ← Clara's development branch
  └── dev/daniel          ← Daniel's development branch
```

### 4.2  Rules

| Rule                                  | Detail                                            |
|---------------------------------------|---------------------------------------------------|
| No direct pushes to `main`            | All changes via PR from `dev/*` branches          |
| One feature per branch                | If a dev branch is cluttered, create a topic branch |
| Build must pass before merge          | `colcon build` + `colcon test` green              |
| PR requires at least 1 review         | Any team member may review                        |
| Squash merge preferred                | Keeps `main` history clean                        |

### 4.3  Commit Conventions

We follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short description>

[optional body]

[optional footer(s)]
```

**Types:**

| Type       | Usage                                     |
|------------|-------------------------------------------|
| `feat`     | New feature                               |
| `fix`      | Bug fix                                   |
| `refactor` | Code restructuring (no behaviour change)  |
| `test`     | Adding or updating tests                  |
| `docs`     | Documentation only                        |
| `chore`    | Build, CI, tooling changes                |
| `perf`     | Performance improvement                   |

**Scopes:** `nav`, `explore`, `dock`, `delivery`, `perception`, `launcher`,
`mission`, `hardware`, `report`

**Examples:**
```
feat(dock): add fallback staging offset on Nav2 rejection
fix(delivery): prevent double-fire during cooldown window
docs(report): add G2 systems design documentation
refactor(nav): consolidate scaffold packages into CDE2310_AMR_Trial_Run + auto_explore_v2
chore(bringup): add Gazebo maze world and mission sim launch
```

---

## 5  AI Attribution Policy

This project uses AI assistants (Claude, GitHub Copilot) for code generation,
documentation, and review. All AI-assisted contributions are disclosed:

| Disclosure Method           | Detail                                              |
|-----------------------------|-----------------------------------------------------|
| Commit trailer              | `Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>` |
| Commit trailer              | `ai-assisted: yes`                                  |
| CHANGELOG                   | AI-assisted entries noted where applicable           |
| Code comments               | Not required for AI-generated code (commits suffice)|

The team retains full responsibility for reviewing, testing, and approving all
AI-generated content before it enters the codebase.

---

## 6  Versioning

The project uses [Semantic Versioning](https://semver.org/) (SemVer):

```
MAJOR.MINOR.PATCH
```

| Component | Incremented When                                  |
|-----------|---------------------------------------------------|
| MAJOR     | Breaking changes to mission interface or launch    |
| MINOR     | New subsystem, new node, or new capability         |
| PATCH     | Bug fixes, parameter tweaks, documentation         |

All changes are recorded in `CHANGELOG.md` at the repository root, following
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) format.

---

## 7  Dependencies

### 7.1  Team-Authored ROS 2 Packages

| Package                 | Build        | Machine | Purpose                                                    |
|-------------------------|--------------|---------|------------------------------------------------------------|
| `auto_explore_v2`       | ament_python | Laptop  | BFS frontier detection + scored Nav2 goal posting          |
| `CDE2310_AMR_Trial_Run` | ament_python | Laptop + RPi | Mission coordinator, docking, search, delivery server |
| `apriltag_docking`      | ament_cmake  | RPi     | C++ perception pipeline: camera_ros → image_proc → apriltag_ros → dock-target TF + PoseStamped publishers |

### 7.2  Upstream ROS 2 Dependencies

| Dependency                   | Version       | Consumed By              | Purpose                            |
|------------------------------|---------------|--------------------------|-------------------------------------|
| `rclpy`                     | Humble        | auto_explore_v2, CDE2310_AMR_Trial_Run | ROS 2 Python client library |
| `rclcpp`                    | Humble        | apriltag_docking          | ROS 2 C++ client library            |
| `std_msgs`                  | Humble        | auto_explore_v2, CDE2310_AMR_Trial_Run | String, Bool message types |
| `std_srvs`                  | Humble        | auto_explore_v2, CDE2310_AMR_Trial_Run | SetBool, Empty service types |
| `geometry_msgs`             | Humble        | all three                 | Twist, PoseStamped, Transform       |
| `nav_msgs`                  | Humble        | auto_explore_v2, CDE2310_AMR_Trial_Run | OccupancyGrid, Odometry, Path |
| `sensor_msgs`               | Humble        | apriltag_docking          | Image, CameraInfo                   |
| `nav2_msgs`                 | Humble        | auto_explore_v2, CDE2310_AMR_Trial_Run | NavigateToPose, ComputePathToPose |
| `nav2_simple_commander`     | Humble        | apriltag_docking          | Nav2 helper API                     |
| `tf2_ros`                   | Humble        | CDE2310_AMR_Trial_Run, apriltag_docking | TF2 buffer, listener, broadcaster |
| `tf2_geometry_msgs`         | Humble        | CDE2310_AMR_Trial_Run     | TF2 message conversions             |
| `action_msgs`               | Humble        | CDE2310_AMR_Trial_Run     | GoalStatus                          |
| `apriltag_ros`              | Humble        | apriltag_docking (upstream) | Tag36h11 detection + pose (composable node) |
| `apriltag_msgs`             | Humble        | apriltag_docking, CDE2310_AMR_Trial_Run | AprilTagDetectionArray    |
| `image_proc`                | Humble        | apriltag_docking          | Composable Resize + Rectify nodes   |
| `image_view`                | Humble        | apriltag_docking          | Image debug utilities               |
| `camera_ros`                | Humble        | apriltag_docking (launch) | RPi CSI camera driver               |
| `turtlebot3_bringup`        | Humble        | RPi bringup               | Robot hardware bringup              |
| `cartographer_ros`          | Humble        | full_mission launch       | SLAM                                |
| `nav2_bringup`              | Humble        | full_mission launch       | Navigation stack launch             |

### 7.3  Python Dependencies

| Package     | Version   | Purpose                             |
|-------------|-----------|--------------------------------------|
| `numpy`     | ≥ 1.21    | Array operations for maps/frontiers  |
| `RPi.GPIO`  | ≥ 0.7 (RPi-only) | GPIO PWM for MG90 servo in `delivery_server` |
| `pytest`    | ≥ 7.0     | Testing framework (lint/style suites)|

Note: AprilTag detection is performed by the C++ `apriltag_ros` composable
node wrapped inside `apriltag_docking`; there is no Python `apriltag`
dependency in the mission code.

### 7.4  System Dependencies

| Package             | Purpose                                      |
|---------------------|----------------------------------------------|
| `python3-colcon-common-extensions` | Build system                    |
| `ros-humble-cartographer-ros` | SLAM                               |
| `ros-humble-nav2-bringup` | Navigation stack                       |
| `ros-humble-v4l2-camera` | Camera driver for RPi Camera V2         |
| `libraspberrypi-dev` | GPIO access on RPi                          |

---

## 8  Build & Test Commands

```bash
# Full build
cd ~/amr_ws
colcon build --symlink-install

# Build single package
colcon build --packages-select CDE2310_AMR_Trial_Run

# Run all tests
colcon test
colcon test-result --verbose

# Lint check
colcon test --packages-select auto_explore_v2  # runs flake8 + pep257
```

---

## 9  Launch Files

| Launch File                             | Package               | Description                              |
|-----------------------------------------|-----------------------|------------------------------------------|
| `auto_explore.launch.py`               | auto_explore_v2       | Cartographer + Nav2 + frontier exploration|
| `mission.launch.py`                    | CDE2310_AMR_Trial_Run | Coordinator + docking + delivery + search|
| `full_mission.launch.py`               | CDE2310_AMR_Trial_Run | Everything: SLAM + Nav2 + mission nodes  |
| `minimal_nav2.launch.py`               | CDE2310_AMR_Trial_Run | Minimal Nav2 for testing                 |
| `gazebo_mission.launch.py`             | CDE2310_AMR_Trial_Run | Gazebo simulation with full mission      |
| `apriltag_dock_pose_publisher.launch.py` | apriltag_docking    | RPi: camera_ros + image_proc + apriltag_ros composable pipeline + dock-target TFs |

---

## 10  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
