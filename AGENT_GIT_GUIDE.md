# Agent Git Guide — Group7_AMR

Quick reference for AI assistants (Copilot, Claude, ChatGPT) working in this repo.

## Golden Rules

1. NEVER commit to `main`. Use `dev/<name>` branches only.
2. NEVER commit `build/`, `install/`, `log/`, `__pycache__/`, `.env`.
3. ALWAYS use conventional commit format.
4. ALWAYS add `ai-assisted: yes` footer when AI helped write the code.

## Branch Workflow

- Pull latest main before starting: `git pull origin main`
- Work on `dev/<name>` only (dev/jeon, dev/shashwat, dev/kuga, dev/clara, dev/daniel)
- Push to your branch, open PR targeting `main`, tag @jeonwonje as reviewer

## Commit Format

```
<type>(<scope>): <short description>

[optional body]

ai-assisted: yes
```

Types: feat, fix, docs, test, chore, refactor
Scopes: nav, explore, dock, delivery, perception, launcher, mission, hardware, report

## Colcon Workspace Rules

- Repo root IS the colcon workspace. Run `colcon build` from repo root.
- ROS2 packages live under `src/`. Non-ROS files in `hardware/`, `docs/`, `data/`.
- Incremental build: `colcon build --packages-select <pkg>`
- After build: `source install/setup.bash`
- COLCON_IGNORE files keep non-ROS dirs out of the build.

## Package Map

| Package | Scope | Contains |
|---|---|---|
| CDE2310_AMR_Trial_Run | mission | Mission coordinator FSM, docking, delivery, search, launcher, shooter nodes |
| auto_explore_v2 | explore | BFS frontier detection, scored Nav2 goal posting |

## Git Worktrees (for reviewers)

```bash
# Create a review worktree
git worktree add ~/worktrees/dev-clara dev/clara
cd ~/worktrees/dev-clara && colcon build && source install/setup.bash

# Remove when done
git worktree remove ~/worktrees/dev-clara
```

## PR Checklist

- [ ] Branch is up to date with main
- [ ] `colcon build` passes
- [ ] Commit messages follow conventional format
- [ ] AI-assisted footer present if AI was used
- [ ] CHANGELOG.md updated
