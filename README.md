# Group 7 — Autonomous Mobile Robot (AMR)

> CDE2310 Engineering Systems Design · AY25/26 Semester 2 · NUS EDIC

A **TurtleBot3 Burger (MeowthBot)** ROS 2 Humble workspace for the CDE2310 warehouse delivery mission. The robot autonomously explores a maze using LiDAR-based SLAM, detects AprilTag delivery stations, docks via geometric visual servoing, and delivers ping-pong balls using a servo-driven rack-and-pinion launcher.

---

## System Design Document

The full SDD is split across eight files under [`docs/`](docs/) — one per G2 section. Start with the ConOps, skim the HLD, and jump into the subsystem / ICD detail as needed.

| § | Document | Covers |
|---|---|---|
| 1 | [Requirements Specification](docs/reports/requirements_specification.md) | FRs, NFRs, constraints, traceability |
| 2 | [Concept of Operations](docs/reports/conops.md) | Mission overview, operational scenarios, design selection, risks |
| 3 | [High-Level Design](docs/reports/high_level_design.md) | Architecture, data flow, hardware-software mapping, design decisions |
| 4 | [Subsystem Design](docs/reports/subsystem_design.md) | Navigation, perception, docking, delivery, mission coordination, search |
| 5 | [Interface Control Document](docs/reports/interface_control_document.md) | ROS 2 topics / services / actions, TF tree, hardware interfaces, network |
| 6 | [Software Development](docs/reports/software_development.md) | Environment, repo layout, branch policy, conventions, dependencies |
| 7 | [Testing](docs/reports/testing_documentation.md) | Strategy, integration tests, FAT, traceability, known issues |
| 8 | [End User Documentation](docs/end_user_doc/EndUserDocumentation_Group7.md) | Printed 5-page operator hand-in for the final mission |

### Hardware & Manufacturing

- [Hardware README](hardware/README.md) — mechanical design, CAD version history, materials (maintained by Daniel).
- [Manufacturing Guide](docs/MANUFACTURING.md) — Bambu Lab P2S print workflow for the team's `.3mf` files.

### Project Meta

- [`CHANGELOG.md`](CHANGELOG.md) — release history (SemVer).
- [`CLAUDE.md`](CLAUDE.md) — repo conventions for AI assistants.
- [`AGENT_GIT_GUIDE.md`](AGENT_GIT_GUIDE.md) — quick AI git reference.

---

## Quick start

```bash
# one-time build
source /opt/ros/humble/setup.bash
cd ~/Group7_AMR
colcon build
source install/setup.bash

# RPi (on the robot)
ros2 launch apriltag_docking apriltag_dock_pose_publisher.launch.py &
ros2 run CDE2310_AMR_Trial_Run delivery_server &
ros2 launch turtlebot3_bringup robot.launch.py

# Laptop (base station, same network)
export TURTLEBOT3_MODEL=burger
ros2 launch CDE2310_AMR_Trial_Run full_mission.launch.py
```

See the [End User Documentation](docs/end_user_doc/EndUserDocumentation_Group7.md) §2 for the full deployment sequence and FAT checklist.

---

## Team

| Member | Role | Branch |
|---|---|---|
| Jeon Won Je | Lead — final system integration, nav tuning, manufacturing | `dev/jeon` |
| Shashwat Gupta | Mission coordinator, docking, perception, Navigation | `dev/shashwat` |
| Kuga | Navigation and Exploration | `dev/kuga` |
| Clara Ong | Delivery server, perception | `dev/clara` |
| Daniel Yow | Mechanical subsystem (hardware, CAD, manufacturing docs) | `dev/daniel` |

---

*CDE2310 · NUS College of Design and Engineering · AY25/26*
