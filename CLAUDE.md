# Contributor workflow

Working notes for anyone — human or agent — continuing this repository.

---

## RULE 0 — IRONCLAD: `/home/pmkhoi/ros_test_ws` IS READ-ONLY

**Never write to, build in, delete from, or modify anything under
`/home/pmkhoi/ros_test_ws`.** It is the live main project, it is not under version
control, and there is no undo.

Permitted against that path: `cat`, `grep`, `diff`, `find`, `cp` *from*, `rsync -a` *from*.

Forbidden there: `colcon build`, any edit, any delete, `git init`, and tidying — do **not**
remove `.bak` files or dead code at the source, however tempting.

To experiment, mirror it first:

```bash
rsync -a --exclude build --exclude install --exclude log \
  /home/pmkhoi/ros_test_ws/  <scratch>/ros_test_ws_mirror/
```

Cleanup happens on the copy **already inside this repo**, after `cp`.

Two independent safeguards exist: `~/chat-pdf-intake` hard-codes
`PROTECTED_ROOT = /home/pmkhoi/ros_test_ws` behind a default-deny allowlist, and every
session ends by re-hashing `ros_test_ws/src` against a baseline manifest.

---

## What this project is

A simulation-only Franka Panda digital twin that executes validated natural-language
pick-and-place commands. Full roadmap, functional requirements (FR-1…10), benchmark suite
(B1–B5) and acceptance thresholds: [`docs/PROJECT-SPEC.md`](docs/PROJECT-SPEC.md).

## The two-repo model (deliberate — do not collapse it)

| Repo | Role |
|---|---|
| `/home/pmkhoi/ros_test_ws` | Read-only scratch workspace. Holds the **final** state of all 5 packages. |
| this repo | Curated portfolio. Receives **one capability per session.** |

Moving code across in increments is what produces a credible commit history. Bulk-copying
the workspace in one commit would destroy the entire point.

### Why per-step commits are even possible

The workspace only holds final state — but its files carry their own earlier revisions as
commented-out blocks:

| File | Preserved history |
|---|---|
| `llm_move.py` | 511 lines, only 314–511 live; banners mark **V.1** and **V.2 (SingleThreadedExecutor)** |
| `move_joint_server.py` | ~75 commented lines of the original single-threaded version |
| `panda_controllers.yaml` | ~30 commented lines of an earlier `joint_position_controller` |
| `panda_bringup.launch.py` | commented-out `jpc_spawner` block |

So stripping dead code and reconstructing intermediate commits are **the same job**: remove
a block from the final file, and use it as the content of the earlier commit.

---

## Step ledger

Narrative source is the session handoff docs, **not** chat PDFs (only Step 1 was ever
exported). Handoffs 1–12 live in `~/Downloads/Project ROS/`; handoff 13 is
`~/Downloads/13__ROS2-PANDA-HANDOFF.md`.

| Step | Handoff | Capability | Status |
|---|---|---|---|
| 1 | `1. ROS2-ACTION-HANDOFF.md` | ROS 2 action lifecycle on a simulated joint | **published** |
| 2 | `2. ROS2-GAZEBO-HANDOFF.md` | Gazebo bring-up, one-joint arm, `ros_gz` bridge | pending |
| 3 | `3. ROS2-CONTROLLER-HANDOFF.md` | Controller-backed action server (MultiThreadedExecutor) | pending |
| 4 | `4. ROS2-PANDA-HANDOFF.md` | Panda URDF + `ros2_control` | pending |
| 5 | `5. ROS2-PANDA-HANDOFF.md` | — confirm from handoff | pending |
| 6 | `6. ROS2-PANDA-HANDOFF.md` | — confirm from handoff | pending |
| 7 | `7. ROS2-PANDA-HANDOFF.md` | — confirm from handoff | pending |
| 8 | `8. ROS2-PANDA-HANDOFF.md` | — confirm from handoff | pending |
| 9 | `9. ROS2-PANDA-HANDOFF.md` | MoveIt config (Setup Assistant) | pending |
| 10 | `10. ROS2-PANDA-HANDOFF.md` | `move_group` against Gazebo (`use_sim_time` last-wins) | pending |
| 11 | `11. ROS2-PANDA-HANDOFF.md` | `move_to.py` — MoveItPy motion primitives | pending |
| 12 | `12. ROS2-PANDA-HANDOFF.md` | `llm_move.py` — Ollama tool-calls + bounds clamp | pending |
| 13 | `13__ROS2-PANDA-HANDOFF.md` | `/voice_command` topic, shared dispatch, motion lock | pending |

Capabilities for steps 5–8 are provisional; confirm each against its handoff before porting.
**Flip the row to `published` in the same commit range that lands the step.**

---

## Per-step pipeline

Take the lowest-numbered `pending` step. Do the whole pipeline, or none of it.

```bash
# 0. Read that step's handoff doc — narrative source of truth.

# 1. Copy this step's files out of the read-only workspace.
cp /home/pmkhoi/ros_test_ws/src/<pkg>/...  src/<pkg>/...

# 2. Clean the copy HERE (never at the source):
#    strip dead blocks, drop .bak files, fix hardcoded paths, fill in package.xml.

# 3. Build and test in an ISOLATED shell (see hazard below).
env -i HOME="$HOME" PATH=/usr/bin:/bin bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /home/pmkhoi/language-controlled-panda-digital-twin
  colcon build && source install/setup.bash &&
  colcon test && colcon test-result --verbose'

# 4. Runtime proof for this step (gz sim / ros2 topic echo / ros2 launch / llm_move run).
# 5. Write docs/learning/step-NN-<slug>.md, mirroring step-01's structure.
# 6. Update README.md so it states current reality.
# 7. Commit in capability-sized commits; flip the ledger row; push.
git push origin main
```

### Build-environment hazard — read before trusting any build

`~/.bashrc` sources `/opt/ros/jazzy/setup.bash` **and `~/ros_test_ws/install/setup.bash`**.
In a normal interactive shell the `twin_*` packages therefore resolve from the *scratch
workspace overlay*, so a build or run here can appear to succeed while actually exercising
`ros_test_ws` artifacts.

Every verification must run in a shell that sources **only** `/opt/ros/jazzy/setup.bash`.
Otherwise "it builds" is not evidence.

### Optional: chat PDF → Markdown

Only if a step's PDF has been exported from claude.ai:

```bash
cp "~/Downloads/Project ROS/Step N_ ... - Claude Chat.pdf" ~/chat-pdf-intake/inbox/
~/chat-pdf-intake/.venv/bin/chat-pdf-intake convert inbox/
```

Output lands in `~/chat-pdf-intake/output/markdown/` and is gitignored there. Prose is
faithful; **code is not** — no fenced blocks survive and `#` comments become headings.
Never copy code out of converted Markdown; always take it from the workspace files.

---

## Conventions

**Commits** — Conventional Commits, sized to a *complete idea*, following the Step 1
boundaries:

```
docs: define digital twin scope and acceptance criteria
feat(interfaces): define single-joint motion action
feat(actions): implement simulated single-joint action server
feat(actions): add asynchronous MoveJoint client
docs: document the ROS 2 action learning milestone
```

Do **not** pad history into equal daily counts. Fewer, meaningful commits read better than
manufactured activity.

**Learning docs** — mirror `docs/learning/step-01-ros2-actions.md`: Objective · Outcome ·
Session progression · Files and what they teach · Debugging lessons · Verification ·
Commit boundaries · Next engineering step.

**Honesty bar.** This is what makes the repo credible:

- Document limitations plainly. Step 1 admits its cancel callback cannot be serviced under
  a single-threaded executor. That paragraph signals more than a page of green checkmarks.
- Record dead ends and the debugging that resolved them.
- Verification tables carry **real observed output**, never claimed output.
- Never describe a capability as working until it has been run in an isolated shell.

**Metadata** — all packages are Apache-2.0 (matching `LICENSE`); maintainer email is the
GitHub noreply address below. No `TODO:` placeholders in `package.xml`.

---

## Environment facts

| | |
|---|---|
| OS / ROS | Ubuntu 24.04, ROS 2 **Jazzy** (`/opt/ros/jazzy`) |
| Simulator | Gazebo Sim **8.11.0** (Harmonic) via `ros_gz`; helper `~/gz_env.sh` sets `GZ_VERSION`, re-implemented inline in `panda_bringup.launch.py` |
| Planning | MoveIt 2, `moveit_py`, `gz_ros2_control` |
| LLM | **local Ollama** at `http://localhost:11434`, model `qwen2.5:3b-instruct`. No API keys anywhere. |
| GitHub | `ptchpmk0100`, `gh` CLI authenticated, https credential helper |
| Git identity | `Khoi Minh Pham <59853829+ptchpmk0100@users.noreply.github.com>` (repo-local; not set globally) |

## Known issues to fix as their step lands

1. **`panda.urdf:359` hardcodes** `/home/pmkhoi/ros_test_ws/install/twin_description/share/...`
   in `<parameters>`. Breaks on every other machine — resolve via the launch file or a
   `.xacro` with `$(find twin_description)`. Highest-priority portability fix.
2. `twin_description` declares **no runtime dependencies** despite requiring `gz_ros2_control`.
3. Undocumented external deps: Ollama + the `qwen2.5:3b-instruct` pull,
   `moveit_resources_panda_description`, `ros_gz`, `moveit_py`.
4. Empty template dirs `twin_{description,interfaces}/{src,include/}`.
5. README currently disclaims features that now exist and cites a cancellation limitation
   fixed in Step 3 — keep it truthful as each step lands.
