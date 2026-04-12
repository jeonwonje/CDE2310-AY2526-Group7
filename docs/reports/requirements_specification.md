# Requirements Specification

| Field          | Value                                              |
|----------------|----------------------------------------------------|
| Document ID    | AMR-RS-001                                         |
| Version        | 1.0                                                |
| Date           | 2026-04-13                                         |
| Author(s)      | Group 7 — Jeon, Kumaresan, Clara, Shashwat, Daniel |
| Module         | CDE2310 Engineering Systems Design                 |
| Status         | Baselined for G2                                   |

---

## 1  Purpose

This document defines the functional, non-functional, and constraint requirements
for the Group 7 Autonomous Mobile Robot (AMR). Every requirement is traceable to
the Concept of Operations (ConOps) and informs the verification plan in the
Testing Documentation.

---

## 2  Scope

The AMR shall autonomously explore a closed maze environment, detect AprilTag
delivery stations, dock at each station, and deliver ping-pong balls within a
single 25-minute mission window. The robot platform is a TurtleBot3 Burger with
a custom spring-loaded launcher payload.

---

## 3  Definitions & Abbreviations

| Term        | Definition                                          |
|-------------|-----------------------------------------------------|
| AMR         | Autonomous Mobile Robot                             |
| AprilTag    | Fiducial marker system (family tag36h11)            |
| Nav2        | ROS 2 Navigation Stack                              |
| SLAM        | Simultaneous Localisation and Mapping               |
| FSM         | Finite State Machine                                |
| DDS         | Data Distribution Service (ROS 2 middleware)        |
| FAT         | Factory Acceptance Test                             |
| ConOps      | Concept of Operations                               |
| RPi         | Raspberry Pi 4B                                     |
| LiDAR       | Light Detection and Ranging (LDS-02 on TurtleBot3)  |

---

## 4  Functional Requirements

### 4.1  Exploration

| ID       | Requirement                                                                 | Traces to |
|----------|-----------------------------------------------------------------------------|-----------|
| FR-EXP-01 | The AMR shall build an occupancy grid map of the maze using Cartographer SLAM. | ConOps §5.3 |
| FR-EXP-02 | The AMR shall detect frontier cells (boundary between explored free space and unknown space) via BFS flood-fill. | ConOps §5.3 |
| FR-EXP-03 | The AMR shall cluster frontiers and score them by a weighted combination of BFS distance and cluster size. | ConOps §5.3 |
| FR-EXP-04 | The AMR shall autonomously navigate to the highest-scored frontier goal until no frontiers remain. | ConOps §5.3 |
| FR-EXP-05 | The AMR shall report EXPLORATION_COMPLETE when all reachable frontiers have been visited. | ConOps §5.3 |

### 4.2  Detection

| ID       | Requirement                                                                 | Traces to |
|----------|-----------------------------------------------------------------------------|-----------|
| FR-DET-01 | The AMR shall detect tag36h11 AprilTag markers using an on-board RPi Camera V2 and the `apriltag` library. | ConOps §5.4 |
| FR-DET-02 | The AMR shall compute the 6-DOF pose of each detected marker via solvePnP and publish the result as a TF transform. | ConOps §5.4 |
| FR-DET-03 | The mission coordinator shall monitor the TF tree for target tags (`tag36h11:0`, `tag36h11:2`) with a staleness threshold of 0.5 s. | ConOps §5.4 |

### 4.3  Docking

| ID       | Requirement                                                                 | Traces to |
|----------|-----------------------------------------------------------------------------|-----------|
| FR-DCK-01 | The AMR shall navigate to a staging point 0.60 m in front of the detected tag using Nav2 NavigateToPose. | ConOps §5.5 |
| FR-DCK-02 | The AMR shall execute a discrete geometric visual-servoing sequence (intercept → square-up → final plunge) to dock within 0.10 m of the tag. | ConOps §5.5 |
| FR-DCK-03 | If docking fails, the coordinator shall blacklist the tag for 30 s and resume exploration. | ConOps §5.5 |
| FR-DCK-04 | The docking server shall abort if the total docking time exceeds 180 s. | ConOps §5.5 |

### 4.4  Delivery

| ID       | Requirement                                                                 | Traces to |
|----------|-----------------------------------------------------------------------------|-----------|
| FR-DEL-01 | At a **static station** (tag36h11:0), the AMR shall fire 3 balls in a timed sequence (fire → 4 s wait → fire → 6 s wait → fire). | ConOps §5.6 |
| FR-DEL-02 | At a **dynamic station** (tag36h11:2), the AMR shall detect the passing target (tag ID 3), fire on detection, and repeat up to 3 shots with a 4 s cooldown. | ConOps §5.6 |
| FR-DEL-03 | The launcher shall use a `/fire_ball` service (std_srvs/Trigger) to activate the servo-driven mechanism. | ConOps §5.6 |

### 4.5  Mission Coordination

| ID       | Requirement                                                                 | Traces to |
|----------|-----------------------------------------------------------------------------|-----------|
| FR-MSN-01 | A central FSM shall orchestrate exploration, docking, delivery, undocking, and search phases. | ConOps §5.2 |
| FR-MSN-02 | The FSM shall transition to SEARCHING when exploration is complete but un-serviced tags remain. | ConOps §5.7 |
| FR-MSN-03 | The FSM shall report MISSION_COMPLETE when all target tags have been serviced. | ConOps §5.8 |

---

## 5  Non-Functional Requirements

| ID       | Category    | Requirement                                                        |
|----------|-------------|--------------------------------------------------------------------|
| NFR-01   | Timing      | The full mission shall complete within 25 minutes.                 |
| NFR-02   | Timing      | Tag detection latency shall not exceed 100 ms per frame at 10 Hz. |
| NFR-03   | Accuracy    | Docking lateral error (Y-offset) shall be ≤ 0.02 m at final stop. |
| NFR-04   | Accuracy    | Docking yaw error shall be ≤ 0.05 rad (≈ 3°) at final stop.      |
| NFR-05   | Reliability | The system shall tolerate camera dropout for up to 1.0 s during docking without aborting. |
| NFR-06   | Reliability | The system shall handle Nav2 goal rejection by applying a fallback staging offset (−0.15 m). |
| NFR-07   | Power       | The RPi and launcher shall operate from the TurtleBot3 12 V battery via buck converter for the full mission duration. |
| NFR-08   | Comms       | DDS discovery between RPi and laptop shall use FastDDS unicast (no multicast). |

---

## 6  Constraints

| ID     | Constraint                                                                     |
|--------|--------------------------------------------------------------------------------|
| CON-01 | Mission duration: 25 minutes maximum.                                          |
| CON-02 | Maze dimensions: provided by NUS EDIC at competition time; walls ≥ 0.3 m high.|
| CON-03 | Delivery payload: exactly 3 ping-pong balls per mission.                       |
| CON-04 | Platform: TurtleBot3 Burger (ROBOTIS); no structural modification to base.     |
| CON-05 | Software: ROS 2 Humble on Ubuntu 22.04 (RPi and laptop).                      |
| CON-06 | Communication: Wi-Fi link between RPi and laptop; no tethered connection.      |
| CON-07 | Autonomy: no manual teleoperation after mission start.                         |

---

## 7  Requirement Traceability Matrix

| Req ID     | ConOps §  | Design §       | Test §        |
|------------|-----------|----------------|---------------|
| FR-EXP-01  | 5.3       | HLD §3.1       | TST-INT-01    |
| FR-EXP-02  | 5.3       | SSD §2.1       | TST-UT-01     |
| FR-EXP-03  | 5.3       | SSD §2.1       | TST-UT-01     |
| FR-DET-01  | 5.4       | SSD §2.2       | TST-INT-02    |
| FR-DET-02  | 5.4       | SSD §2.2       | TST-INT-02    |
| FR-DCK-01  | 5.5       | SSD §2.3       | TST-SYS-01    |
| FR-DCK-02  | 5.5       | SSD §2.3       | TST-SYS-01    |
| FR-DEL-01  | 5.6       | SSD §2.4       | TST-SYS-02    |
| FR-DEL-02  | 5.6       | SSD §2.4       | TST-SYS-02    |
| FR-MSN-01  | 5.2       | SSD §2.5       | TST-SYS-03    |
| FR-MSN-02  | 5.7       | SSD §2.5       | TST-SYS-03    |

---

## 8  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
