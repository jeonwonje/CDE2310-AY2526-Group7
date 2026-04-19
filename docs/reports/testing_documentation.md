# Testing Documentation

| Field          | Value                                              |
|----------------|----------------------------------------------------|
| Document ID    | AMR-TST-001                                        |
| Version        | 1.0                                                |
| Date           | 2026-04-13                                         |
| Author(s)      | Group 7 — Jeon, Kumaresan, Clara, Shashwat, Daniel |
| Module         | CDE2310 Engineering Systems Design                 |
| Status         | Baselined for G2                                   |

---

## 1  Purpose

This document defines the test strategy, test cases, and known defects for
the Group 7 AMR. It covers unit, integration, and system-level testing aligned
with the requirements in AMR-RS-001.

---

## 2  Test Strategy

| Level           | Scope                                    | Tools               | Automated? |
|-----------------|------------------------------------------|----------------------|------------|
| Unit            | Individual functions / algorithms         | pytest               | Yes        |
| Lint / Style    | PEP 8 compliance, docstrings             | flake8, pep257       | Yes        |
| Integration     | Node-to-node communication (topics, svc) | Manual + ros2 CLI    | Partial    |
| System (FAT)    | Full mission end-to-end                  | Physical maze run    | No         |
| Simulation      | Gazebo world smoke tests                 | Gazebo + RViz        | No         |

---

## 3  Unit & Lint Tests

Unit coverage is currently limited to the ament lint / style scaffolds that
ship inside each Python package. The mission stack does not yet include
algorithm-level unit tests. All shipped tests run via `colcon test` and are
reported through `colcon test-result --verbose`.

| Package               | Test files                                          | Checks                        |
|-----------------------|-----------------------------------------------------|-------------------------------|
| auto_explore_v2       | `test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` | Copyright header, flake8, pep257 |
| CDE2310_AMR_Trial_Run | `test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` | Copyright header, flake8, pep257 |
| apriltag_docking      | `ament_lint_auto` + `ament_lint_common` (via `package.xml`) | C++ lint suite                |

**Known gap.** Deeper unit coverage for the pathfinding / frontier scoring
logic (`auto_explore_v2`), docking geometry (`docker.py`), and mission FSM
transitions (`mission_coordinator_v3.py`) is planned but not yet implemented.
Algorithm-level validation is currently performed through the integration and
system tests in §4 and §5.

---

## 4  Integration Test Plan

### 4.1  Test Cases

| ID         | Title                              | Preconditions                        | Steps                                                       | Expected Result                        | Traces to  |
|------------|------------------------------------|--------------------------------------|-------------------------------------------------------------|----------------------------------------|------------|
| TST-INT-01 | Exploration publishes frontiers    | Cartographer running, /map available | 1. Launch `auto_explore.launch.py`<br>2. Monitor `frontiers` topic | Frontier JSON published within 5 s     | FR-EXP-02  |
| TST-INT-02 | AprilTag TF broadcast              | RPi camera running, tag visible      | 1. `ros2 launch apriltag_docking apriltag_dock_pose_publisher.launch.py` on RPi<br>2. Hold tag in front of camera<br>3. `ros2 run tf2_ros tf2_echo camera tag36h11:0` | Transform printed with age < 0.5 s | FR-DET-02  |
| TST-INT-03 | toggle_exploration service         | score_and_post running               | 1. `ros2 service call toggle_exploration SetBool "{data: false}"`<br>2. Verify no new goal_pose<br>3. Re-enable with `{data: true}` | Goals pause and resume correctly        | FR-MSN-01  |
| TST-INT-04 | delivery_server fires servo via GPIO | delivery_server running on RPi       | 1. Send START_DELIVERY command<br>2. Observe MG90 servo activation on GPIO 12 | Servo fires, ball launched             | FR-DEL-03  |
| TST-INT-05 | mission_command → docker           | docker node running                  | 1. Publish `START_DOCKING` JSON on `/mission_command`<br>2. Monitor docker logs | Docker enters STAGING state             | FR-DCK-01  |
| TST-INT-06 | DDS cross-machine topic flow       | Both machines on same Wi-Fi          | 1. RPi: publish test String on /test_topic<br>2. Laptop: echo /test_topic | Message received within 1 s            | NFR-08     |

### 4.2  Integration Test Execution

Integration tests are performed manually during lab sessions. Results are
recorded in the test log (Section 7).

---

## 5  System Test Plan (Factory Acceptance Test)

### 5.1  FAT Checklist

The FAT simulates a full mission run in the lab maze. Each item must pass for
the system to be considered mission-ready.

| ID         | Criterion                                   | Pass Condition                                     | Traces to       |
|------------|---------------------------------------------|----------------------------------------------------|-----------------|
| TST-SYS-01 | Full maze exploration                      | ≥ 90% occupancy grid coverage within 15 min        | FR-EXP-01–05   |
| TST-SYS-02 | Tag detection during exploration           | Both station tags detected and TF published         | FR-DET-01–03   |
| TST-SYS-03 | Docking at Station A                       | Robot stops within 0.10 m, Y error ≤ 0.03 m        | FR-DCK-01–02   |
| TST-SYS-04 | Static delivery (3 balls)                  | 3 balls fired at Station A                          | FR-DEL-01      |
| TST-SYS-05 | Docking at Station B                       | Same criteria as TST-SYS-03                         | FR-DCK-01–02   |
| TST-SYS-06 | Dynamic delivery (target tag)              | 3 balls fired reactively on dynamic tag detection             | FR-DEL-02      |
| TST-SYS-07 | Mission completion                         | Coordinator reaches MISSION_COMPLETE state          | FR-MSN-03      |
| TST-SYS-08 | Total time ≤ 25 min                        | Wall-clock time from start to MISSION_COMPLETE      | NFR-01         |
| TST-SYS-09 | No manual intervention                     | Operator does not touch robot or keyboard post-start| CON-07         |
| TST-SYS-10 | Blacklist recovery                         | If dock fails, robot resumes exploration, retries later | FR-DCK-03  |
| TST-SYS-11 | Search fallback                            | If tag missed during exploration, search phase finds it | FR-MSN-02  |
| TST-SYS-12 | Emergency stop                             | LiDAR stop_distance (0.25 m) prevents wall collision | NFR-05        |

---

## 6  Known Issues & Defects

References: GitHub Issues #5–#8

| Issue # | Title                                          | Severity | Status     | Component           | Description                                                                                 |
|---------|------------------------------------------------|----------|------------|---------------------|---------------------------------------------------------------------------------------------|
| #5      | Camera flicker causes false tag detections     | Medium   | Mitigated  | apriltag_docking / mission_coordinator | Rapid lighting changes cause frame-level false positives. Mitigated by stale TF threshold (0.5 s). |
| #6      | Nav2 goal rejection near map boundaries        | Medium   | Mitigated  | docker.py           | Staging waypoints near map edges sometimes rejected. Mitigated by fallback_staging_offset.   |
| #7      | Time offset drift between RPi and laptop       | Low      | Open       | System-wide         | Manual ~0.40 s offset occasionally drifts. No NTP configured. Stale TF threshold absorbs most drift. |
| #8      | Feed tube indexing unreliable after 6th ball    | Low      | Open       | launcher (mechanical) | Gravity-fed tube occasionally jams near the last ball. Not mission-critical (7-ball capacity with 6 needed). |

---

## 7  Test Results Template

### 7.1  Lint / Style Test Log

| Date       | Tester | Package               | flake8 | pep257 | copyright | Notes |
|------------|--------|-----------------------|--------|--------|-----------|-------|
| 2026-04-XX | —      | auto_explore_v2       | —      | —      | —         |       |
| 2026-04-XX | —      | CDE2310_AMR_Trial_Run | —      | —      | —         |       |

### 7.2  Integration Test Log

| Date       | Tester | Test ID     | Result      | Notes                          |
|------------|--------|-------------|-------------|--------------------------------|
| 2026-04-XX | —      | TST-INT-01  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-02  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-03  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-04  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-05  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-06  | PASS / FAIL |                                |

### 7.3  System Test (FAT) Log

| Date       | Tester | FAT Run # | TST-SYS-01 | TST-SYS-02 | TST-SYS-03 | TST-SYS-04 | TST-SYS-05 | TST-SYS-06 | TST-SYS-07 | TST-SYS-08 | TST-SYS-09 | TST-SYS-10 | TST-SYS-11 | TST-SYS-12 | Overall | Notes |
|------------|--------|-----------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|---------|-------|
| 2026-04-XX | —      | 1         | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —       |       |

---

## 8  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
