# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - 2026-04-17

### Changed
- Consolidated all SDD documentation into a single README (simple, not simplistic)
- README now covers all G2 required sections: ConOps, requirements, high-level design, subsystem design, interface control, software development, testing, end user documentation

### Added
- Semver release tags
- Gazebo simulation launch file with full mission support
- README architecture diagrams and launch instructions

### Fixed
- Launcher node: retract plunger on node shutdown (Ctrl-C)
- Launcher node: restored service architecture after delivery+shooter merge revert

## [1.1.0] - 2026-04-13

### Added
- G2 systems engineering documentation (`docs/reports/`):
  - Requirements specification (AMR-RS-001)
  - Concept of Operations / ConOps
  - High-level design with two-machine architecture
  - Subsystem design for all 6 subsystems
  - Interface control document (full ROS 2 topic/service/action catalogue)
  - Software development plan
  - Testing documentation (29 unit tests, integration plan, FAT checklist)
- End user documentation (`docs/end_user_doc/`)
- `amr_nav` package: custom Dijkstra/A* navigation with 29 unit tests

### Changed
- Consolidated 5 packages → 3 (`amr_nav`, `amr_perception`, `amr_launcher`)
- Navigation no longer depends on nav2_msgs, nav2_bringup, or py_trees

### Removed
- `amr_navigation` package (replaced by `amr_nav`)
- `amr_bringup` package (merged into `amr_nav`)
- Legacy Nav2 parameter files

## [1.0.0] - 2026-03-15

### Added
- `auto_explore_v2` package: BFS frontier detection with clustering, scored goal posting via Nav2 (Kumaresan)
- `CDE2310_AMR_Trial_Run` package: mission coordinator FSM, search stations, AprilTag detector (Kumaresan)
- Docking server — discrete geometric visual servoing for AprilTag docking (Shashwat)
- Launcher node and RPi shooter node — servo PWM control (Clara)
- Delivery server — ball delivery orchestration with cooldown management (Jeon)
- Cartographer SLAM config, minimal Nav2 params, full mission launch file
- Complete TurtleBot3 assembly CAD model (`hardware/chassis/`)
- Launcher mechanism CAD — carousel, barrel, plunger, spur gear (`hardware/launcher/`)
- 3MF manufacturing files for 3D printing
- Repository structure: `src/`, `hardware/`, `docs/`, `data/`, `archive/`
- CLAUDE.md coding standards and AGENT_GIT_GUIDE.md
