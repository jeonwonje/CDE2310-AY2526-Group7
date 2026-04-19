# CLAUDE.md — Group7_AMR

Coding standards and repo conventions for AI assistants working in this repository.
See also: [AGENT_GIT_GUIDE.md](AGENT_GIT_GUIDE.md)

## Project Overview

ROS 2 Humble colcon workspace for the CDE2310 Autonomous Mobile Robot.
The robot explores an arena, detects AprilTag targets, and delivers a
ping-pong ball via a spring-loaded launcher.

## Folder Structure

```
Group7_AMR/                ← colcon workspace root
├── src/
│   ├── CDE2310_AMR_Trial_Run/  ← Mission coordination, docking, delivery, search, shooter
│   └── auto_explore_v2/        ← BFS frontier detection, scored Nav2 goals
├── hardware/
│   ├── launcher/          ← Launcher mechanism CAD
│   └── chassis/           ← TurtleBot3 assembly + mounts
├── docs/
│   ├── reports/           ← SDD reports (detailed versions)
│   └── end_user_doc/      ← End-user documentation
├── data/                  ← Recorded bags, maps
├── CLAUDE.md              ← This file
├── AGENT_GIT_GUIDE.md     ← Quick reference for AI agents
├── CHANGELOG.md           ← Keep-a-changelog format
├── README.md              ← Consolidated SDD + project overview
└── requirements.txt       ← pip dependencies
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Group7_AMR
colcon build                              # full build
colcon build --packages-select <pkg>      # incremental
source install/setup.bash                 # overlay
```

## Branch Policy

| Branch | Purpose |
|---|---|
| `main` | Protected. Merges via PR only. |
| `dev/jeon` | Jeon Won Je — lead; final integration, nav tuning, manufacturing |
| `dev/shashwat` | Shashwat — mission coordinator, docking, perception |
| `dev/kuga` | Kuga — navigation core |
| `dev/clara` | Clara — delivery server, perception |
| `dev/daniel` | Daniel — mechanical subsystem |

**Never commit directly to main.** Always work on `dev/<name>`, then open a PR.

## Commit Convention

```
<type>(<scope>): <short description>

[optional body]

ai-assisted: yes          ← required when AI helped write the code
```

### Types
`feat` `fix` `docs` `test` `chore` `refactor`

### Scopes
`nav` `perception` `launcher` `chassis` `report` `conops`

### Examples
```
feat(navigation): add frontier cost function
fix(perception): handle missing camera frame
docs(report): update milestone 2 figures
chore(bringup): add slam launch file
```

## PR Process

1. Ensure `colcon build` passes.
2. Update `CHANGELOG.md` under `[Unreleased]`.
3. Open PR targeting `main`, tag `@jeonwonje` as reviewer.
4. PR description must summarize changes and link related issues.

## AI Attribution

When AI (Copilot, Claude, ChatGPT) generates or substantially modifies code:
- Add `ai-assisted: yes` as a commit message footer.
- Add a brief inline comment if a non-obvious algorithm was AI-generated.

## CHANGELOG Format

Use [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) under the
`[Unreleased]` section. Categories: Added, Changed, Fixed, Removed.

## Code Style

- Python: follow PEP 8, enforced by `ament_flake8`.
- ROS2 nodes: use `rclpy` with lifecycle patterns where appropriate.
- Launch files: Python launch files in each package's `launch/` directory.
- Config files: YAML in each package's `config/` directory.

## Files Never to Commit

`build/` `install/` `log/` `__pycache__/` `.env` `*.bag` `*.db3`
