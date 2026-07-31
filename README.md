# Language-Controlled Panda Digital Twin

An incremental ROS 2 robotics project whose end goal is a simulation-only
Franka Panda manipulator that executes validated natural-language
pick-and-place commands in Gazebo.

> **Current status:** actuated simulation, no autonomy yet. The real 7-DOF Panda
> spawns into Gazebo with valid physics, is anchored to the world, and holds or
> tracks a position command on every arm joint. On a one-joint stand-in, the full
> loop is closed end to end: a ROS 2 action goal drives the joint through a PID
> controller and reports the *measured* angle back as feedback, brought up by a
> single launch file. The Panda does not yet have a ROS-side command path, and the
> repository contains no MoveIt planning, perception, or language-model
> integration.

## Milestones

| Step | Capability | Documentation |
|---|---|---|
| 1 | Custom ROS 2 action on a simulated numerical joint | [Step 1](docs/learning/step-01-ros2-actions.md) |
| 2 | Gazebo physics + real joint state bridged into ROS 2 | [Step 2](docs/learning/step-02-gazebo-pipeline.md) |
| 3 | Closed loop: the action commands the joint, measured state is the feedback | [Step 3](docs/learning/step-03-joint-controller.md) |
| 4 | The real 7-DOF Panda spawns with valid physics, anchored to the world | [Step 4](docs/learning/step-04-panda-spawn.md) |
| 5 | All seven Panda arm joints hold pose and track a commanded angle | [Step 5](docs/learning/step-05-panda-controllers.md) |

The one-joint arm is a deliberate stand-in. Steps 2 and 3 proved the
description → physics → bridge → action pipeline on the smallest robot that could
exercise it; Steps 4 and 5 point that same pipeline at the real arm.

## Step 5: Seven-joint position control

The Panda's seven revolute joints each carry a `JointPositionController` plugin,
and the model carries a single `JointStatePublisher`:

```text
7 × JointPositionController   per JOINT — one command topic each
1 × JointStatePublisher       per MODEL — reports every joint in one message
```

They are not matched pairs, which is the thing to get right: eight `<gazebo>`
blocks, not fourteen. The arm now holds the pose it is put in rather than folding
under gravity, and tracks commanded angles to within 0.03 rad. See
[Step 5: Panda controllers](docs/learning/step-05-panda-controllers.md).

## Step 4: The Panda in physics

The MoveIt Panda description ships **12 links and 0 `<inertial>` blocks**, because
it is built for motion planning rather than simulation. Gazebo reports that as
"a model must have at least one link" — a consequence, not a description, since a
massless dynamic link is invisible to a physics engine.

The repair is to expand `panda.urdf.xacro`, which carries authentic Franka CAD
mass properties, rather than hand-authoring twelve fabricated inertial blocks. A
`world` link and a fixed joint then anchor the base. See
[Step 4: Panda spawn](docs/learning/step-04-panda-spawn.md).

## Step 3: Closed-loop joint control

An action goal becomes a real command to a real controller, and the feedback the
client prints is measured by the physics engine rather than computed in Python:

```text
move_joint_client 0.8
  → MoveJoint goal
  → server publishes std_msgs/Float64 on .../joint1/cmd_pos
  → ros_gz_bridge (ROS → gz)
  → JointPositionController plugin drives joint1 under real physics
  → JointStatePublisher plugin → ros_gz_bridge (gz → ROS)
  → server publishes the MEASURED angle as action feedback
```

This step also introduced the reverse bridge direction, a launch file that
collapses four terminals into two, and the executor design that makes cancellation
reachable during motion. See
[Step 3: Closed-loop joint control](docs/learning/step-03-joint-controller.md).

## Step 2: Gazebo joint-state pipeline

A hand-written one-joint arm is spawned into Gazebo Harmonic, simulated with real
physics, and its measured joint state is bridged into ROS 2:

```text
URDF (every link carries <inertial>)
  → spawned into Gazebo physics
  → joint1 swings under gravity
  → JointStatePublisher plugin broadcasts on a gz topic
  → ros_gz_bridge converts gz.msgs.Model → sensor_msgs/JointState
  → /world/empty/model/one_joint_arm/joint_state readable in ROS 2
```

Nothing drove the joint at this stage — the arm fell and settled near π/2, which is
correct for an unpowered link. The MoveIt Panda description was tried first and
rejected by Gazebo for having no `<inertial>` blocks; that failure is what motivated
building a minimal arm to prove the pipeline, and Step 4 is where it was finally
resolved. See [Step 2: Gazebo pipeline](docs/learning/step-02-gazebo-pipeline.md).

## Step 1: ROS 2 action lifecycle

The first milestone separates the action contract from its Python
implementation:

```text
MoveJoint client
      │ target angle
      ▼
MoveJoint server ──► simulated joint state
      │
      ├── streamed position/error feedback
      └── success result or out-of-range rejection
```

It demonstrates:

- a custom goal/result/feedback interface generated with ROSIDL;
- server-side bounds checking for targets outside ±3.14 radians;
- a stateful action server that simulates incremental motion;
- an asynchronous Python client using chained futures and feedback callbacks;
- clean separation between an `ament_cmake` interface package and an
  `ament_python` node package.

The session analysis and engineering lessons are documented in
[Step 1: ROS 2 actions](docs/learning/step-01-ros2-actions.md). The full
planned system and acceptance criteria are in
[PROJECT-SPEC.md](docs/PROJECT-SPEC.md).

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon`
- Gazebo Harmonic (Gazebo Sim 8) and `ros-jazzy-ros-gz` — required from Step 2
- `ros-jazzy-moveit-resources-panda-description` — required from Step 4, for the
  Panda's meshes

Gazebo is installed through ROS `-vendor` packages, which place it inside the ROS
tree rather than on the system `PATH`. Sourcing ROS does **not** configure it. A
terminal running any `gz` command must first export `GZ_VERSION`, `GZ_CONFIG_PATH`,
`LD_LIBRARY_PATH`, and add `gz_tools_vendor/bin` to `PATH`; see
[Step 2](docs/learning/step-02-gazebo-pipeline.md#environment-lessons). Terminals
running only ROS nodes do not need this.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Inspect the generated action:

```bash
ros2 interface show twin_interfaces/action/MoveJoint
```

## Run

### One-joint arm, closed loop (Steps 1–3)

Two terminals. The first needs the Gazebo environment; the second does not.

```bash
# Terminal 1 — the simulator. Press play: the controller's PID loop only runs
# while physics is stepping, so a paused world makes every goal time out.
gz sim empty.sdf
```

```bash
# Terminal 2 — spawn, both bridge directions, and the action server.
ros2 launch twin_action_demo arm_bringup.launch.py
```

The spawn node exits as soon as its request is accepted; that is a one-shot node
finishing, not a crash. Then command the arm from a third sourced terminal:

```bash
ros2 run twin_action_demo move_joint_client 0.8      # radians, positional
ros2 run twin_action_demo move_joint_client          # defaults to 1.57
ros2 run twin_action_demo move_joint_client --ros-args -p target_angle:=1.0
```

Feedback streams the measured angle as the arm moves, and the goal succeeds once
the measurement is within tolerance of the target. Exercise the bounds check with
a target outside ±3.14 rad:

```bash
ros2 run twin_action_demo move_joint_client 5.0      # rejected before motion
```

### Panda (Steps 4–5)

The Panda has no launch file or ROS-side command path yet — that is the next step.
Spawn it directly, with an **absolute** `-file` path, because the path is resolved
by the already-running simulator rather than by your shell:

```bash
gz sim empty.sdf          # press play
ros2 run ros_gz_sim create -world empty -name panda -z 0.0 \
  -file "$(ros2 pkg prefix twin_description)/share/twin_description/urdf/panda.urdf"
```

All seven joints hold the spawn pose. Command one directly over gz transport,
which proves the controllers without involving a bridge at all:

```bash
gz topic -l | grep cmd_pos                                    # expect seven
gz topic -t /panda_joint4/cmd_pos -m gz.msgs.Double -p 'data: -1.5'
```

To read the joints back in ROS 2, bridge the model's state topic:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/empty/model/panda/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
ros2 topic echo /world/empty/model/panda/joint_state
```

## Test

```bash
colcon test
colcon test-result --verbose
```

Validated on ROS 2 Jazzy and Gazebo Sim 8 (11 tests, 0 failures, 1 skipped):

- clean build of all three packages;
- CMake and package XML checks passed;
- Python Flake8 and PEP 257 checks passed;
- `one_joint_arm.urdf` and `panda.urdf` both parse; the Panda's root link is
  `world`, confirming the anchor;
- one-joint arm, headless: goals of 0.80, 1.57 and −1.00 rad each succeeded and
  settled within 0.02 rad of target; a 5.00 rad goal was rejected before anything
  was published;
- Panda, headless: model spawns with no inertial rejection, and its base pose is
  unchanged at `[0 0 0]` after five seconds of stepping physics;
- Panda joints hold the zero spawn pose instead of folding — five of seven within
  0.005 rad, wrist joints 5 and 7 within 0.072 rad — and track commands to within
  0.03 rad, with no drift across repeated samples at rest.

## Known limitations

**The Panda cannot be commanded from ROS 2.** Its joints are driven over gz
transport only. The seven-way bridge and an action server that publishes seven
targets and reports seven-joint feedback are the next step; until then, only the
one-joint arm has a closed ROS-side loop.

**Wrist joints hold less accurately than the inner joints.** `panda_joint5` and
`panda_joint7` settle with roughly 0.05–0.07 rad of offset, against under 0.005 rad
for most of the arm. They have a 12 N·m effort limit rather than 87 N·m and carry
the welded hand mass.

**Gains are baked into the descriptions**, so every tuning change costs an edit and
a `colcon build`. Externalising them into YAML is one of the reasons the roadmap
moves to `ros2_control`.

**The Panda description is generated and then hand-edited.** It is produced by
expanding the upstream xacro, after which the world anchor and the plugin blocks
are added by hand; re-running `xacro` discards them. The file header records the
command and the hazard.

**The Panda's mesh paths are absolute** into `/opt/ros/jazzy/...`, so the
description assumes a Jazzy install at the default prefix rather than being
portable across machines.

**Motion is uncoordinated and unchecked.** The seven joints take independent scalar
setpoints; nothing sequences them into a synchronised trajectory, and nothing
enforces joint limits before a command is published. Self-collision is off, which
is the Gazebo default and intentional — inter-link collision belongs at the
planning layer, which arrives with MoveIt.

**The gripper is inert.** Its mimic constraint is unsupported by the DART physics
engine, and nothing drives the fingers.

**Cancellation is reachable but untested.** Step 3 replaced the single-threaded
executor with a `MultiThreadedExecutor` and a `ReentrantCallbackGroup`, so a cancel
request can now be serviced while a goal is executing — but no automated test
covers it.

## Repository layout

```text
.
├── docs/
│   ├── PROJECT-SPEC.md
│   └── learning/            one entry per milestone, step-01 … step-05
└── src/
    ├── twin_interfaces/     MoveJoint action definition (ament_cmake)
    ├── twin_action_demo/    action server, client, and bringup launch file
    └── twin_description/    one_joint_arm.urdf and panda.urdf
```

## Development

This project is built as a guided learning exercise, with AI pair-programming
(Claude) used for teaching, review, and documentation. The engineering decisions,
debugging, and verification recorded in `docs/learning/` are the substance of the
work; each milestone is built and run before it is documented.

Where re-running a milestone contradicted the notes taken during it, the
measurement wins and the discrepancy is recorded — see the correction in Step 3 and
the rejected experiment in Step 5.

Generated `build/`, `install/`, and `log/` trees are intentionally excluded
from version control.
