# PROJECT-SPEC — Language-Controlled Robotic Manipulator Digital Twin

> Executable specification for the project roadmap.
> Status: **proposed / not yet built.** Nothing in this document asserts that the
> project has already achieved these skills or results. All numeric thresholds are
> **targets and estimates**, not measured values.
>
> Locked design decisions: **Gazebo (Harmonic) primary simulator — Isaac Sim dropped
> (hardware below its minimum)** · local PyTorch perception · **local small LLM** for
> language → command · **cloud GPU for ML training only**.
>
> Re-targeted 2026-07-28: the original spec assumed Ubuntu 22.04 + ROS 2 Humble + Isaac Sim.
> The build machine runs **Ubuntu 24.04** and does **not** meet Isaac Sim's minimum
> (needs RTX / ~8 GB VRAM; available card is an entry ~4 GB dGPU), so the stack is now
> **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**, with cloud GPU reserved for ML
> training. The architecture, tool schema, requirements, benchmarks, and metrics are
> simulator-agnostic and are unchanged.

---

## 1. Executive Summary & Scope

Build a **simulation-only digital twin of a 7-DOF Franka Emika Panda arm** that executes
**tabletop pick-and-place** tasks issued in **plain natural language**. A local small
language model turns a sentence into a validated sequence of restricted robot tool-calls;
MoveIt 2 plans the motion; Gazebo executes it; every command and outcome is logged to
SQLite and scored against a fixed benchmark suite.

The Panda is chosen deliberately: it has first-class Gazebo + MoveIt 2 assets, is the
de-facto research standard, and extends prior work on robotic-arm control rather than
introducing an unrelated application domain.

**In scope**

- Single fixed-base Panda arm in simulation.
- Tabletop scene: a table, a bin/plate/target, and 3–6 known primitive objects (colored
  blocks, cup, marker).
- Natural-language commands → restricted tool-calls → planned, collision-checked motion.
- Perception of object identity + pose (ground-truth first, trained detector later).
- SQLite logging + a quantitative benchmark + a technical report + a demo video.

**Out of scope (explicitly)**

- Real hardware / sim-to-real transfer.
- Mobile base, navigation, multi-arm, bimanual, or dexterous/in-hand manipulation.
- Deformable objects, liquids, cloth.
- Human-in-the-loop teleoperation.

**MVP (must ship)** — Sections 4 + 6 define "done":
Language command → Panda performs a single pick-and-place in Gazebo, unsafe commands
are rejected, and every command is logged. Benchmark B1 runs end-to-end with numbers.

**Stretch goals (only after MVP is green)**

- Trained synthetic-RGB detector replacing ground-truth poses.
- Multi-step + sequential language (B3, B5).
- LLM-vs-rule-based ablation study.
- Isaac Sim / higher-fidelity render port — **only if better hardware becomes available**
  (gated, not expected; Gazebo remains the primary and demo target).

---

## 2. Target Tech Stack (concrete)

| Layer                | Choice                                                                             | Notes                                                                      |
| -------------------- | ---------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| OS                   | Ubuntu 24.04 LTS (native)                                                          | Native LTS matched to ROS 2 Jazzy; Gazebo needs no RTX GPU                  |
| Middleware           | **ROS 2 Jazzy**                                                                    | LTS matched to Ubuntu 24.04                                                |
| Motion planning      | **MoveIt 2** (`ros-jazzy-moveit`)                                                  | Provides IK, planning, collision checking (a safety dependency)            |
| Simulator (primary)  | **Gazebo Harmonic** + `ros_gz` bridge                                             | Panda SDF/URDF, depth camera, world randomization; runs on entry GPU/CPU   |
| Simulator (alt)      | **PyBullet / MuJoCo**                                                              | Lightweight scripting fallback only if Gazebo friction blocks progress     |
| Perception           | **PyTorch** + sim RGB-D                                                            | MVP = sim ground-truth poses; upgrade = synthetic-RGB detector (train on cloud, infer locally) |
| Language agent       | **Local small instruction model** — **Qwen2.5-1.5B-Instruct** default (tiny tool-call vocabulary → 1.5B + grammar is ample, fits 4 GB); 3B Q4 as a quality stretch | Served via Ollama or `transformers`; emits **constrained JSON** tool-calls; cloud GPU as fallback for larger-model comparison |
| Structured output    | JSON schema / grammar-constrained decoding                                         | Guarantees parseable tool-calls                                            |
| Logging              | **SQLite**                                                                         | One row per command; see §6                                                |
| Eval / plots         | Python + pandas + matplotlib                                                       | Benchmark harness in `eval/`                                               |
| Cloud (ML only)      | Colab / rented GPU VM                                                              | Detector *training* + larger-LLM comparison; **no cloud simulation**       |
| Language / VCS       | Python 3.12, Git + GitHub                                                          | Public portfolio repo (Ubuntu 24.04 / Jazzy default Python)               |

The local-LLM choice is intentional over a cloud API: it demonstrates on-device model
serving, quantization, and constrained decoding — all higher-signal for robotics/embodied-AI
roles than calling a hosted endpoint.

---

## 3. Architecture

```
                        ┌───────────────────────────────────────────────┐
  natural-language      │                                               │
  command  ───────────► │  Local small LLM  (constrained JSON tool-call)│
  "put the red block    │  Qwen2.5-1.5B (3B Q4 stretch)                  │
   in the bin"          └───────────────┬───────────────────────────────┘
                                        │  tool-call(s)
                                        ▼
                        ┌───────────────────────────────────────────────┐
                        │  Command Validator / Safety Layer             │
                        │  • schema check   • workspace bounds          │
                        │  • object exists? • collision precheck        │  ── reject ──► refusal + log
                        └───────────────┬───────────────────────────────┘
                                        │  validated command
                                        ▼
     ┌──────────────┐        ┌──────────────────────┐        ┌───────────────────────┐
     │ Perception   │  pose  │  MoveIt 2 planner     │ traj   │  Gazebo (Panda)       │
     │ (poses of    │ ─────► │  IK + plan + collide  │ ─────► │  executes trajectory  │
     │  objects)    │        └──────────────────────┘        └──────────┬────────────┘
     └──────┬───────┘                                                   │ state / result
            │  RGB-D                                                    ▼
            └────────────────────────────────────────────►  ┌───────────────────────┐
                                                             │  SQLite logger        │
                                                             └──────────┬────────────┘
                                                                        ▼
                                                             ┌───────────────────────┐
                                                             │  Evaluator / metrics  │
                                                             └───────────────────────┘
```

**The "restricted tool-using agent."** The LLM may emit **only** these verbs — nothing
else is executable:

| Tool      | Signature          | Meaning                                         |
| --------- | ------------------ | ----------------------------------------------- |
| `move_to` | `move_to(x, y, z)` | Move end-effector to a workspace coordinate     |
| `pick`    | `pick(object_id)`  | Grasp a named, currently-detected object        |
| `place`   | `place(target_id)` | Place held object at/relative to a named target |
| `home`    | `home()`           | Return to the home pose                         |
| `stop`    | `stop()`           | Halt immediately                                |

Any output that is off-schema, references an absent object, or lands outside the workspace
box is rejected by the validator **before** any motion — and the refusal is logged as a
first-class outcome (this is what makes B4 measurable).

---

## 4. Functional Requirements

Each requirement has a **pass condition** that can be checked in simulation.

| ID    | Requirement                                                      | Pass condition                                                                                      |
| ----- | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| FR-1  | Parse a natural-language command into a valid tool-call sequence | For a held-out command set, ≥ target% produce schema-valid calls matching intended tools/args       |
| FR-2  | Reject any off-schema or malformed model output                  | 100% of malformed outputs are caught; no malformed call reaches the planner                         |
| FR-3  | Enforce workspace bounds                                         | Any target outside the defined workspace box is rejected before planning                            |
| FR-4  | Collision-checked planning                                       | Every executed trajectory passes MoveIt 2 collision checking; no plan executes if planning fails    |
| FR-5  | Execute a single pick-and-place                                  | Named object ends inside the named target region, gripper released                                  |
| FR-6  | Resolve object identity → pose                                   | Correct object selected and localized (ground-truth in MVP; detector later)                         |
| FR-7  | Multi-step sequencing                                            | A command implying N sub-actions executes them in correct order (stretch: B3/B5)                    |
| FR-8  | Refuse impossible / ambiguous commands                           | For B4 (object absent / referent ambiguous), the agent refuses and does not move                    |
| FR-9  | Log every command                                                | One SQLite row per command with all §6 fields; 0 dropped commands                                   |
| FR-10 | Deterministic fallback interface                                 | The tool schema can be driven directly (no LLM) for Phase-1 bring-up and as the rule-based baseline |

---

## 5. Benchmark Task Suite (fixed)

Run **20 randomized trials per benchmark** (object start positions randomized within the
table each trial — domain randomization). The suite is frozen so numbers are comparable
across the 90 days and across the LLM-vs-baseline ablation.

| ID     | Task                     | Example command                           | Tests                             |
| ------ | ------------------------ | ----------------------------------------- | --------------------------------- |
| **B1** | Single pick-and-place    | "Put the red block in the bin."           | FR-1,5,6,9 (this is the MVP gate) |
| **B2** | Relative placement       | "Place the cup to the left of the plate." | Spatial-relation grounding        |
| **B3** | Multi-step stack         | "Stack the blue block on the red block."  | FR-7 sequencing                   |
| **B4** | Negative / ambiguous     | "Pick up the green block." (none present) | FR-8 refusal — must **not** move  |
| **B5** | Sequential multi-command | "First go home, then pick the marker."    | FR-7 ordering across verbs        |

---

## 6. Metrics & Acceptance Criteria

**SQLite row schema (per command):**
`id, timestamp, benchmark_id, trial, raw_text, parsed_calls_json, validated (bool),
rejection_reason, planning_success (bool), planning_time_ms, execution_success (bool),
end_to_end_latency_ms, safety_violation (bool)`.

**Metrics computed by the evaluator:**

- Command-parse accuracy (parsed calls match intent)
- Per-benchmark task-success rate
- Grasp success rate
- Planning success rate / mean planning time
- End-to-end latency (text received → first motion)
- **Safety violations** (out-of-bounds or collision attempts that reached execution)
- Robustness = success-rate spread across randomized trials

**MVP acceptance thresholds** (targets — the definition of "done" for the core):

| Criterion                              | Target       |
| -------------------------------------- | ------------ |
| Parse accuracy on benchmark vocabulary | ≥ 80%        |
| B1 end-to-end success                  | ≥ 60%        |
| B4 correct refusals                    | ≥ 90%        |
| Safety violations reaching execution   | **0** (hard) |
| Commands logged                        | 100% (hard)  |

Hard criteria (0 violations, 100% logging) are non-negotiable; percentage targets may be
revised once real numbers exist, and any revision is recorded in the report.

---

## 7. 30 / 60 / 90-Day Plan

Three phases, each de-risked so the project retains a working increment before adding
the next hard piece. One shippable artifact per week.

### Phase 1 — Foundation (Days 1–30): make the arm move, deterministically

Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 stand up; Panda spawned in the
tabletop scene; **scripted** pick-and-place from hardcoded poses; SQLite logging live; the §3 tool
schema drives the arm **without any LLM** (this becomes both the Phase-1 demo and the
Phase-3 rule-based baseline). _Phase deliverable:_ scripted pick-place **video** + repo
skeleton + README.

### Phase 2 — Language + Perception (Days 31–60): make it understand

Local small model integrated behind constrained JSON decoding → tool-calls; validator /
safety layer (FR-2,3,8) enforced; **cloud GPU (Colab / rented VM) stood up** for detector
training + larger-LLM comparison; perception path online — **ground-truth poses first**,
then a synthetic-RGB detector (trained on cloud, inferred locally) if time allows;
benchmark harness runs B1–B3.
_Phase deliverable:_ language-driven pick-place **demo** + first eval numbers.

### Phase 3 — Evaluation + Polish (Days 61–90): make it measurable & presentable

Full B1–B5 at 20 trials with randomization; metrics + plots generated from SQLite;
**ablation** LLM vs rule-based baseline (same tool interface, FR-10); technical report,
demo video, GitHub polish, tagged release. _Phase deliverable:_ report + metric tables +
demo video + `v1.0` tag.

### Weekly cadence (12 weeks, one artifact each)

| Wk  | Focus                                                     | Shippable artifact                                   |
| --- | --------------------------------------------------------- | ---------------------------------------------------- |
| 1   | Ubuntu 24.04 + ROS 2 Jazzy + Gazebo + MoveIt install; Panda spawns | Screencast: Panda visible & jogging in Gazebo        |
| 2   | Tabletop scene + objects + depth-camera sensor placed     | World file + screenshot                              |
| 3   | MoveIt 2 pick-place from hardcoded poses                  | Video: scripted single pick-place                    |
| 4   | Tool-schema command interface + SQLite logging            | Deterministic tool-call runs a pick-place; DB rows   |
| 5   | Local LLM served (Ollama), small model on 4 GB or cloud endpoint | Notebook: text → JSON tool-call offline       |
| 6   | Constrained decoding + validator/safety layer             | Malformed & out-of-bounds commands rejected + logged |
| 7   | End-to-end: LLM command → Panda motion                    | Video: "put the red block in the bin" works          |
| 8   | Perception: ground-truth poses → object lookup            | B1 runs end-to-end with logged results               |
| 9   | Benchmark harness B1–B3 + 20-trial randomization          | First metrics table (B1–B3)                          |
| 10  | B4 refusals + B5 sequencing; (stretch) synthetic detector trained on cloud, inferred locally | Metrics for B4/B5; refusal behavior demo |
| 11  | Rule-based baseline + LLM-vs-baseline ablation            | Comparison plot                                      |
| 12  | Report + demo video + README + release                    | `PROJECT-REPORT.md`, demo video, `v1.0` tag          |

Phase-1 carries the schedule buffer: if ROS/Gazebo setup slips (the highest-risk item for a
beginner), Week-2/3 scene work absorbs it before the LLM work begins.

---

## 8. Repository / Artifact Layout

```
manipulator-twin/
├── README.md                 # overview, setup, demo GIF, results table
├── ros_ws/
│   └── src/
│       ├── twin_bringup/      # launch files: ros_gz bridge, MoveIt, nodes
│       ├── twin_control/      # tool-schema executor (move_to/pick/place/home/stop)
│       └── twin_msgs/         # command / result message defs
├── sim/
│   ├── worlds/               # Gazebo SDF tabletop world
│   └── models/               # Panda + object models
├── agent/
│   ├── model/                # local LLM serving + quantization config
│   ├── tool_schema.py        # the 5 restricted verbs (JSON schema)
│   ├── validator.py          # schema + workspace + object-exists checks
│   └── rule_based.py         # deterministic baseline (FR-10)
├── perception/
│   ├── ground_truth.py       # MVP pose source from sim
│   └── detector/             # synthetic-RGB detector (stretch)
├── cloud/                    # Colab notebooks + training configs (ML-only, no sim)
├── eval/
│   ├── benchmarks.py         # B1–B5 definitions + trial runner
│   ├── metrics.py            # reads SQLite → metrics
│   └── plots/                # generated figures
├── db/
│   └── runs.sqlite           # command/outcome log
└── docs/
    ├── PROJECT-REPORT.md     # final technical report
    └── media/                # demo video, GIFs
```

---

## 9. Skill → Job-Market Return Map

Each completed capability should produce concrete evidence for relevant engineering roles.

| Skill built                           | Relevant roles                                               |
| ------------------------------------- | ------------------------------------------------------------ |
| ROS 2 (Jazzy)                         | Robotics software, autonomous systems, ROS-based development |
| MoveIt 2 motion planning              | Robot manipulation, robotics software                        |
| Gazebo (Harmonic) simulation          | Robotics simulation, digital twins, embodied AI              |
| PyTorch perception (synthetic data)   | Robot perception, computer vision                            |
| Local-LLM tool-use / restricted agent | Agentic AI, LLM applications, industrial AI                  |
| Safety/validation layer               | Robust robotics and safety-aware AI systems                  |
| SQLite logging + benchmarking         | Data analytics, evaluation-heavy engineering                 |
| Quantitative evaluation & ablation    | Experimental engineering, ML evaluation                      |
| Technical report + demo               | Engineering communication                                   |

---

## 10. Risk Register

| Risk                               | Likelihood | Mitigation                                                                                               |
| ---------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------- |
| Local-LLM latency/VRAM on 4 GB GPU | Med        | 1.5B quantized model + grammar-constrained *short* outputs; cloud endpoint fallback; rule-based fallback (FR-10) keeps the demo alive |
| Cloud-GPU setup / cost for training| Med        | Ground-truth-pose MVP needs **no** GPU; detector training is an explicit stretch, batched on free/low-cost Colab |
| ROS 2 learning curve (beginner)    | High       | Front-loaded into Phase 1; scripted deterministic path before any LLM; lean on MoveIt + Gazebo tutorials  |
| Perception harder than expected    | Med        | Ground-truth poses are the MVP; the trained detector is explicitly a stretch goal, not a blocker         |
| Scope creep / burnout              | Med        | Hard MVP line (§1); one artifact per week; stretch goals gated behind a green MVP                        |

---

## 11. Immediate Next Actions (Week 1)

1. Confirm Ubuntu 24.04 native install + NVIDIA driver (Gazebo needs no RTX).
2. Install ROS 2 Jazzy and MoveIt 2 (`ros-jazzy-desktop`, `ros-jazzy-moveit`); verify with
   the MoveIt Panda demo.
3. Install Gazebo Harmonic + `ros-jazzy-ros-gz`; spawn the Panda; confirm `ros_gz` bridges
   joint states.
4. Create the `manipulator-twin/` repo skeleton (§8) and push an empty-but-structured
   commit with this spec as `docs/`.

> Every claim to be made on a resume from this project must trace to a logged benchmark
> result or a committed artifact. Until then, this remains a plan — not experience.

```

```
