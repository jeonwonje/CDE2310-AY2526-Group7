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

## 3  Unit Tests

*Note: The `amr_nav` package containing 29 pathfinding unit tests was removed
during the package consolidation. The test code and results are preserved in
git history. The following documents the tests as originally designed.*

### 3.1  Test Class: `TestOccupancyToGrid`

Tests the `occupancy_to_grid()` function that converts a ROS OccupancyGrid
flat array into a binary numpy grid.

| Test ID     | Test Name                 | Description                                    | Traces to |
|-------------|---------------------------|------------------------------------------------|-----------|
| TST-UT-01   | `test_basic_conversion`   | Mixed free/wall/unknown cells convert correctly| FR-EXP-01 |
| TST-UT-02   | `test_empty_grid`         | Empty input produces (0,0) grid                | —         |
| TST-UT-03   | `test_all_free`           | All-zero input → all-zero grid                 | FR-EXP-01 |
| TST-UT-04   | `test_all_occupied`       | All-100 input → all-one grid                   | FR-EXP-01 |
| TST-UT-05   | `test_unknown_is_occupied`| Cells with value −1 map to occupied (1)        | FR-EXP-01 |
| TST-UT-06   | `test_custom_threshold`   | Custom wall_threshold boundary works correctly | FR-EXP-01 |

### 3.2  Test Class: `TestFindNextTarget`

Tests the multi-source Dijkstra target finder.

| Test ID     | Test Name                     | Description                                   | Traces to |
|-------------|-------------------------------|-----------------------------------------------|-----------|
| TST-UT-07   | `test_simple_target`          | Finds nearest unvisited cell in open grid     | FR-EXP-04 |
| TST-UT-08   | `test_no_target_all_visited`  | Returns None when entire grid is visited      | FR-EXP-05 |
| TST-UT-09   | `test_no_target_all_walls`    | Returns None when grid is fully occupied      | FR-EXP-05 |
| TST-UT-10   | `test_respects_distance_threshold` | Ignores targets beyond max Dijkstra cost | FR-EXP-04 |
| TST-UT-11   | `test_finds_closest`          | Among multiple targets, picks the nearest     | FR-EXP-04 |
| TST-UT-12   | `test_navigates_around_wall`  | Finds target reachable only via detour        | FR-EXP-04 |
| TST-UT-13   | `test_single_cell_grid`       | Edge case: 1×1 grid                          | —         |
| TST-UT-14   | `test_large_grid_performance` | 200×200 grid completes in < 2 s              | NFR-02    |

### 3.3  Test Class: `TestAstarWallPenalty`

Tests the A* pathfinder with wall proximity penalties.

| Test ID     | Test Name                     | Description                                   | Traces to |
|-------------|-------------------------------|-----------------------------------------------|-----------|
| TST-UT-15   | `test_straight_path`          | Straight path in open corridor                | FR-EXP-04 |
| TST-UT-16   | `test_path_around_obstacle`   | Finds path around a single wall block         | FR-EXP-04 |
| TST-UT-17   | `test_no_path`                | Returns None when target is walled off        | FR-EXP-05 |
| TST-UT-18   | `test_start_equals_goal`      | Returns single-element path                   | —         |
| TST-UT-19   | `test_wall_penalty_biases_centre` | Path prefers corridor centre over wall edge | NFR-03    |
| TST-UT-20   | `test_diagonal_not_allowed`   | 4-connected movement only (no diagonals)      | —         |
| TST-UT-21   | `test_custom_wall_cost`       | Higher wall_cost produces wider berth         | —         |
| TST-UT-22   | `test_narrow_corridor`        | Finds path through 1-cell-wide corridor       | FR-EXP-04 |
| TST-UT-23   | `test_large_grid_astar`       | 200×200 grid completes in < 2 s              | NFR-02    |

### 3.4  Test Class: `TestClusterPath`

Tests the waypoint reduction algorithm.

| Test ID     | Test Name                     | Description                                   | Traces to |
|-------------|-------------------------------|-----------------------------------------------|-----------|
| TST-UT-24   | `test_basic_clustering`       | Removes intermediate waypoints                | FR-EXP-04 |
| TST-UT-25   | `test_empty_path`             | Empty input → empty output                    | —         |
| TST-UT-26   | `test_single_point`           | Single point → single point                   | —         |
| TST-UT-27   | `test_preserves_endpoints`    | Start and goal always retained                | FR-EXP-04 |
| TST-UT-28   | `test_custom_cluster_distance`| Larger distance → fewer waypoints             | —         |
| TST-UT-29   | `test_all_close_together`     | Dense points collapse to start + end          | —         |

### 3.5  Running Unit Tests

```bash
cd ~/amr_ws

# Run all tests
colcon test
colcon test-result --verbose
```

**Expected output (all 29 pass):**

```
test_pathfinding.py::TestOccupancyToGrid::test_basic_conversion       PASSED
test_pathfinding.py::TestOccupancyToGrid::test_empty_grid             PASSED
...
test_pathfinding.py::TestClusterPath::test_all_close_together         PASSED

========================= 29 passed in 1.42s =========================
```

---

## 4  Lint / Style Tests

Each package includes ament standard lint tests:

| Package               | Tests                          | Location                    |
|-----------------------|--------------------------------|-----------------------------|
| auto_explore_v2       | flake8, pep257, copyright     | `test/test_flake8.py` etc. |
| CDE2310_AMR_Trial_Run | flake8, pep257, copyright     | `test/test_flake8.py` etc. |

Run: `colcon test` → `colcon test-result --verbose`

---

## 5  Integration Test Plan

### 5.1  Test Cases

| ID         | Title                              | Preconditions                        | Steps                                                       | Expected Result                        | Traces to  |
|------------|------------------------------------|--------------------------------------|-------------------------------------------------------------|----------------------------------------|------------|
| TST-INT-01 | Exploration publishes frontiers    | Cartographer running, /map available | 1. Launch `auto_explore.launch.py`<br>2. Monitor `frontiers` topic | Frontier JSON published within 5 s     | FR-EXP-02  |
| TST-INT-02 | AprilTag TF broadcast              | Camera running, tag visible          | 1. Launch `apriltag_detector`<br>2. Hold tag in front of camera<br>3. `ros2 run tf2_ros tf2_echo camera_link tag36h11:0` | Transform printed with age < 0.5 s | FR-DET-02  |
| TST-INT-03 | toggle_exploration service         | score_and_post running               | 1. `ros2 service call toggle_exploration SetBool "{data: false}"`<br>2. Verify no new goal_pose<br>3. Re-enable with `{data: true}` | Goals pause and resume correctly        | FR-MSN-01  |
| TST-INT-04 | /fire_ball triggers servo          | rpi_shooter_node running on RPi      | 1. `ros2 service call /fire_ball Trigger`<br>2. Observe servo activation | Servo fires, response.success = true   | FR-DEL-03  |
| TST-INT-05 | mission_command → docker           | docker node running                  | 1. Publish `START_DOCKING` JSON on `/mission_command`<br>2. Monitor docker logs | Docker enters STAGING state             | FR-DCK-01  |
| TST-INT-06 | DDS cross-machine topic flow       | Both machines on same Wi-Fi          | 1. RPi: publish test String on /test_topic<br>2. Laptop: echo /test_topic | Message received within 1 s            | NFR-08     |

### 5.2  Integration Test Execution

Integration tests are performed manually during lab sessions. Results are
recorded in the test log (Section 8).

---

## 6  System Test Plan (Factory Acceptance Test)

### 6.1  FAT Checklist

The FAT simulates a full mission run in the lab maze. Each item must pass for
the system to be considered mission-ready.

| ID         | Criterion                                   | Pass Condition                                     | Traces to       |
|------------|---------------------------------------------|----------------------------------------------------|-----------------|
| TST-SYS-01 | Full maze exploration                      | ≥ 90% occupancy grid coverage within 15 min        | FR-EXP-01–05   |
| TST-SYS-02 | Tag detection during exploration           | Both station tags detected and TF published         | FR-DET-01–03   |
| TST-SYS-03 | Docking at Station A                       | Robot stops within 0.10 m, Y error ≤ 0.02 m        | FR-DCK-01–02   |
| TST-SYS-04 | Static delivery (3 balls)                  | 3 balls fired at Station A                          | FR-DEL-01      |
| TST-SYS-05 | Docking at Station B                       | Same criteria as TST-SYS-03                         | FR-DCK-01–02   |
| TST-SYS-06 | Dynamic delivery (target tag)              | ≥ 1 ball fired on dynamic tag detection             | FR-DEL-02      |
| TST-SYS-07 | Mission completion                         | Coordinator reaches MISSION_COMPLETE state          | FR-MSN-03      |
| TST-SYS-08 | Total time ≤ 25 min                        | Wall-clock time from start to MISSION_COMPLETE      | NFR-01         |
| TST-SYS-09 | No manual intervention                     | Operator does not touch robot or keyboard post-start| CON-07         |
| TST-SYS-10 | Blacklist recovery                         | If dock fails, robot resumes exploration, retries later | FR-DCK-03  |
| TST-SYS-11 | Search fallback                            | If tag missed during exploration, search phase finds it | FR-MSN-02  |
| TST-SYS-12 | Emergency stop                             | LiDAR stop_distance (0.25 m) prevents wall collision | NFR-05        |

---

## 7  Known Issues & Defects

References: GitHub Issues #5–#8

| Issue # | Title                                          | Severity | Status     | Component           | Description                                                                                 |
|---------|------------------------------------------------|----------|------------|---------------------|---------------------------------------------------------------------------------------------|
| #5      | Camera flicker causes false tag detections     | Medium   | Mitigated  | amr_perception      | Rapid lighting changes cause frame-level false positives. Mitigated by stale TF threshold (0.5 s). |
| #6      | Nav2 goal rejection near map boundaries        | Medium   | Mitigated  | docker.py           | Staging waypoints near map edges sometimes rejected. Mitigated by fallback_staging_offset.   |
| #7      | Time offset drift between RPi and laptop       | Low      | Open       | System-wide         | Manual ~0.40 s offset occasionally drifts. No NTP configured. Stale TF threshold absorbs most drift. |
| #8      | Carousel indexing unreliable after 3rd ball    | Low      | Open       | amr_launcher (mech) | Spring return occasionally fails to index carousel to 4th position. Not mission-critical (3-ball limit). |

---

## 8  Test Results Template

### 8.1  Unit Test Log

| Date       | Tester | Test Suite          | Passed | Failed | Skipped | Notes              |
|------------|--------|---------------------|--------|--------|---------|--------------------|
| 2026-04-XX | —      | test_pathfinding.py | —/29   | —      | —       |                    |

### 8.2  Integration Test Log

| Date       | Tester | Test ID     | Result      | Notes                          |
|------------|--------|-------------|-------------|--------------------------------|
| 2026-04-XX | —      | TST-INT-01  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-02  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-03  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-04  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-05  | PASS / FAIL |                                |
| 2026-04-XX | —      | TST-INT-06  | PASS / FAIL |                                |

### 8.3  System Test (FAT) Log

| Date       | Tester | FAT Run # | TST-SYS-01 | TST-SYS-02 | TST-SYS-03 | TST-SYS-04 | TST-SYS-05 | TST-SYS-06 | TST-SYS-07 | TST-SYS-08 | TST-SYS-09 | TST-SYS-10 | TST-SYS-11 | TST-SYS-12 | Overall | Notes |
|------------|--------|-----------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|---------|-------|
| 2026-04-XX | —      | 1         | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —           | —       |       |

---

## 9  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
