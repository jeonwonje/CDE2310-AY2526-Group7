# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Initial colcon workspace skeleton with 5 ament_python packages
- Repository structure: `src/`, `hardware/`, `docs/`, `data/`
- CLAUDE.md coding standards and AGENT_GIT_GUIDE.md for AI assistants
- V1 launcher prototype CAD
- Complete TurtleBot3 assembly model with all four layers, sub-assemblies, and reference parts (`hardware/chassis/assembly/`)
- Custom launcher mechanical components — carousel ball feed, motor clamps, barrel guide, flywheel assembly (`hardware/launcher/components/`) — plunger, tube, barrel-to-storage adapter, servo mount, spur gear (`hardware/launcher/v1/`)
- `amr_nav` package: custom navigation replacing Nav2 stack
  - `pathfinding.py`: multi-source Dijkstra target finding, A* with wall
    penalty, cluster-to-cluster waypoint reduction
  - `auto_nav.py`: single-node state machine for exploration, AprilTag
    interrupt, station approach/alignment, and ball delivery
  - `coverage_monitor.py`: moved from amr_navigation (unchanged)
  - `nav_tuning.yaml`: all tunable navigation constants in one file
  - `sim_exploration.launch.py`: Gazebo sim launch (no Nav2)
  - `mission.launch.py`: full warehouse mission launch (no Nav2)
  - Unit tests for all pathfinding algorithms (29 tests)

### Changed
- Consolidated 5 packages → 3 (`amr_nav`, `amr_perception`, `amr_launcher`)
- Navigation no longer depends on nav2_msgs, nav2_bringup, or py_trees

### Removed
- `amr_navigation` package (replaced by `amr_nav`)
- `amr_bringup` package (merged into `amr_nav`)
- `amr_utils` moved to `archive/` (frozen, not built)
- Nav2 parameter files (`nav2_params.yaml`, `nav2_params_sim.yaml`)
