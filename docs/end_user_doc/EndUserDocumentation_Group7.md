# End User Documentation

## Section 1: General System Description & Critical Data (Spec Sheet)

**Model/Product Name:** TurtleBot 3 Waffle Pi — Group 7

| Specification | Value |
|---|---|
| Dimensions | 281 × 306 × 141 mm (base L×B×H); ~300 mm total with payload |
| Weight | *[Weigh assembled robot and fill in]* |
| Power / Battery | LB-012, 11.1 V 1800 mAh LiPo; ~80 min normal / ~26 min heavy load |
| Drive | Differential drive, 2× Dynamixel XM430-W210 |
| Max Linear Velocity | 0.22 m/s (configurable) |
| LiDAR | LDS-02, 360° scan, 0.12–3.5 m range |
| Onboard Computer | Raspberry Pi 4 |
| Motor Controller | OpenCR 1.0 (9-axis IMU) |
| Camera | Raspberry Pi Camera Module V2 (CSI) |
| Station Detection | AprilTag 36h11 family (tag IDs 0 and 2) |
| Launcher Type | Continuous-rotation servo, rack-and-pinion plunger |
| Launcher Actuator | Servo on GPIO 12 (BCM), 50 Hz PWM, duty 10.0 for CCW |
| Time per Launch Cycle | 0.87 s |
| Ball Capacity | 3 standard 40 mm ping pong balls |

### Description

An autonomous mobile robot that navigates a maze using LiDAR-based SLAM, detects delivery stations via AprilTag markers, and launches ping pong balls into receptacles using a servo-driven plunger mechanism. The system operates as a distributed ROS 2 architecture with a central finite state machine coordinating exploration, docking, and delivery phases.

### Operation Summary

1. **Explore** — Robot autonomously maps the maze using frontier-based exploration. Custom nodes identify unexplored boundaries, score them by wall clearance and distance, preflight paths, then navigate.
2. **Detect** — While exploring, the mission coordinator monitors the TF tree for AprilTag frames (`tag36h11:0` for Station A, `tag36h11:2` for Station B). Detection triggers an immediate transition to docking.
3. **Dock** — A geometric visual servoing node stages the robot 0.60 m from the tag via Nav2, then executes a discrete intercept → square-up → final plunge sequence to reach 0.15 m from the tag.
4. **Deliver** — Station A: fires 3 balls at timed intervals (4 s, 6 s gaps). Station B: detects AprilTag ID 3 on moving rail and fires reactively with cooldown between shots.
5. **Resume** — After delivery, the coordinator resumes exploration or enters search phase if the map is fully explored.

### Common Troubleshooting

| Issue | Resolution |
|---|---|
| Robot does not move | Check OpenCR power switch; reseat USB between RPi and OpenCR |
| LiDAR not spinning | Reconnect LDS-02 USB; verify with `ls /dev/ttyUSB*` |
| "No frontiers found" | Map fully explored or SLAM not running; verify `/map` publishing |
| Exploration times out early | Increase `master_exploration_timeout_sec` param (default 300 s) |
| Nav2 goal keeps timing out | Increase `nav2_goal_timeout_sec` (default 60 s); check costmap |
| AprilTag not detected | Ensure camera CSI cable is seated; check apriltag_ros node running |
| Docking aborts repeatedly | Increase `blacklist_timeout` in coordinator (default 30 s); check lighting |
| Launcher jams | Clear barrel; check rack-gear alignment; lightly sand barrel interior |
| Servo not firing | Verify GPIO 12 connection; check `RPi.GPIO` installed; test with `raspi-gpio` |
| Robot oscillates near walls | Reduce `inflation_radius` in Nav2 config (try 0.40 m vs default 0.55 m) |

### Safety and Precautionary Measures

- Keep fingers clear of launcher barrel and rack-and-pinion mechanism during operation.
- Do not puncture, short-circuit, or over-discharge the LiPo below 9 V. Store in a fireproof bag.
- The robot moves without warning once exploration nodes are active. Maintain a clear perimeter.
- LDS-02 emits Class 1 laser light. Do not stare directly into the sensor aperture.
- **Emergency stop**: Press OpenCR reset button to halt all motor output immediately.

---

## Section 2: Technical Guide

### Software Packages

The system uses two custom ROS 2 packages:

| Package | Purpose | Key Nodes |
|---|---|---|
| `auto_explore_v2` | Frontier exploration + scoring | `find_frontiers`, `score_and_post` |
| `CDE2310_AMR_Trial_Run` | Mission coordination, docking, delivery | `mission_coordinator`, `docking_server`, `delivery_server`, `search_server`, `rpi_shooter_node` |

### Software Setup

1. SSH into the Raspberry Pi:
```
ssh ubuntu@<ROBOT_IP>
```

2. Clone the packages into the TurtleBot3 workspace:
```
cd ~/turtlebot3_ws/src
git clone <REPO_URL>
cd ~/turtlebot3_ws
```

3. Build and source:
```
colcon build
source install/setup.bash
```

4. Dependencies: ROS 2 Humble, Nav2, SLAM Toolbox, TurtleBot3 packages, `apriltag_ros`, `RPi.GPIO`.

### Deployment

1. **Load** 3 ping pong balls into the carousel/magazine.
2. **Power on** the OpenCR board. Wait ~30 s for Raspberry Pi boot.
3. **SSH** into the TurtleBot.
4. **Terminal 1** — TurtleBot3 bringup:
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```
5. **Terminal 2** — SLAM:
```
ros2 launch slam_toolbox online_async_launch.py
```
6. **Terminal 3** — Nav2:
```
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$(ros2 pkg prefix auto_explore_v2)/share/auto_explore_v2/config/nav2_params.yaml
```
7. **Terminal 4** — AprilTag detection:
```
ros2 launch apriltag_ros apriltag.launch.py
```
8. **Terminal 5** — Exploration nodes:
```
ros2 launch auto_explore_v2 auto_explore.launch.py
```
9. **Terminal 6** — Hardware shooter (runs on RPi):
```
ros2 run CDE2310_AMR_Trial_Run rpi_shooter_node
```
10. **Terminal 7** — Mission system:
```
ros2 launch CDE2310_AMR_Trial_Run mission.launch.py
```

**Expected output:** The robot maps the maze, detects AprilTag stations, docks, delivers balls, and resumes until all stations are serviced or exploration timeout is reached.

### Shutdown

1. `Ctrl+C` all terminal sessions.
2. Switch off OpenCR board.
3. Disconnect battery if storing.

---

## Section 3: Acceptable Defect Log

| Defect No. | Defect | Justification |
|---|---|---|
| 1 | Minor surface roughness inside barrel | Acceptable if plunger slides freely without binding |
| 2 | Small layer separation on 3D-printed mounts | Acceptable if no flexing under load; structural integrity maintained |
| 3 | 1–2 missing standoff screws per layer | ≥3 standoffs per layer provides sufficient stability |
| 4 | Minor cosmetic scratches on waffle plates | No functional impact |
| 5 | Slight servo horn play | Acceptable if launch trajectory remains consistent |

---

## Section 4: Factory Acceptance Test (FAT)

| Test Description | Expected Outcome | Test Done |
|---|---|---|
| Teleoperation movement test | Robot responds to all teleop commands correctly | ☐ |
| TurtleBot3 bringup | `/scan`, `/odom`, `/cmd_vel` all publish; no errors | ☐ |
| SLAM mapping | `/map` publishes; walls render correctly in RViz2 | ☐ |
| Frontier exploration | Robot auto-explores; ≥90% area coverage; publishes `EXPLORATION_COMPLETE` when done | ☐ |
| Nav2 path planning | Robot plans and follows paths without collision | ☐ |
| AprilTag detection (static) | `tag36h11:0` detected within 2 m; TF frame published | ☐ |
| AprilTag detection (dynamic) | `tag36h11:2` and tag ID 3 detected during rail pass | ☐ |
| Docking sequence | Robot stages at 0.60 m, servos to 0.15 m; `DOCKING_COMPLETE` published | ☐ |
| Launcher fire test | `/fire_ball` service fires servo; ball exits barrel cleanly; 3/3 successful | ☐ |
| Static delivery (Station A) | 3 balls delivered at timed intervals; all enter receptacle | ☐ |
| Dynamic delivery (Station B) | ≥1 ball delivered to moving receptacle on rail pass | ☐ |
| Full mission dry run | Explore → detect → dock → deliver at both stations within 25 min | ☐ |

---

## Section 5: Maintenance and Part Replacement Log

| Date | Part Replaced | Issue |
|---|---|---|
| | | |
| | | |
| | | |
| | | |
| | | |

---

*Document Version 1.0 — 9 April 2026*
*This document must remain with the robot at all times.*
*Robot Serial: Group 7, CDE2310 AY25/26, NUS EDIC*
