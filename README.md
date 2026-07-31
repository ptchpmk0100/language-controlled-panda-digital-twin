# Language-Controlled Panda Digital Twin

An incremental ROS 2 robotics project whose end goal is a simulation-only
Franka Panda manipulator that executes validated natural-language
pick-and-place commands in Gazebo.

> **Current status:** a trajectory-controlled simulated arm with a planning
> configuration, but no planning yet. One launch command brings up Gazebo, the
> 7-DOF Panda, and the ROS 2 control stack on simulation time, with a
> `JointTrajectoryController` exposing the `FollowJointTrajectory` action MoveIt
> executes through. A MoveIt configuration package exists and loads, but
> `move_group` has not been run against the simulator, so nothing plans, and there
> is no perception or language-model integration.

## Milestones

| Step | Capability | Documentation |
|---|---|---|
| 1 | Custom ROS 2 action on a simulated numerical joint | [Step 1](docs/learning/step-01-ros2-actions.md) |
| 2 | Gazebo physics + real joint state bridged into ROS 2 | [Step 2](docs/learning/step-02-gazebo-pipeline.md) |
| 3 | Closed loop: the action commands the joint, measured state is the feedback | [Step 3](docs/learning/step-03-joint-controller.md) |
| 4 | The real 7-DOF Panda spawns with valid physics, anchored to the world | [Step 4](docs/learning/step-04-panda-spawn.md) |
| 5 | All seven Panda arm joints hold pose and track a commanded angle | [Step 5](docs/learning/step-05-panda-controllers.md) |
| 6 | One-command bringup, a YAML-configured bridge, and a seven-joint action | [Step 6](docs/learning/step-06-panda-bringup.md) |
| 7 | Rearchitected onto `ros2_control`, the interface MoveIt expects | [Step 7](docs/learning/step-07-ros2-control.md) |
| 8 | Simulation time, and a controller that accepts timed trajectories | [Step 8](docs/learning/step-08-sim-time-and-trajectories.md) |
| 9 | The planning layer: MoveIt configuration wired to the real controller | [Step 9](docs/learning/step-09-moveit-config.md) |

The one-joint arm is a deliberate stand-in. Steps 2 and 3 proved the
description → physics → bridge → action pipeline on the smallest robot that could
exercise it; Steps 4 onward point that same pipeline at the real arm.

## Step 9: The planning layer

The URDF and `panda_controllers.yaml` describe how a joint is commanded and read.
A planner needs different facts about the same machine, and those live in
`twin_moveit_config`:

| File | Answers |
|---|---|
| `panda.srdf` | What is an arm? Which link pairs can never collide? |
| `kinematics.yaml` | How is inverse kinematics solved? |
| `joint_limits.yaml` | How fast may a plan move? |
| `moveit_controllers.yaml` | How is a finished plan executed? |

The two layers meet at exactly one string. `moveit_controllers.yaml` sends a
`FollowJointTrajectory` goal to `/arm_controller/follow_joint_trajectory` — the
action Step 8's controller advertises. The Setup Assistant names that controller
after the planning group by default, which is a plausible name for a controller
that does not exist, and a mismatch there plans successfully and then silently
never executes. See
[Step 9: MoveIt configuration](docs/learning/step-09-moveit-config.md).

## Step 8: Simulation time and trajectories

Two interface mismatches stood between the control stack and MoveIt. The
controller manager was not following Gazebo's clock — and fixing that exposed that
`/clock` had **zero publishers**, because Gazebo keeps simulation time internally
and a bridge has to carry it into ROS. The arm was also driven by an untimed
position array, which has nothing to interpolate between.

`JointGroupPositionController` was replaced by `arm_controller`, a
`JointTrajectoryController` that takes waypoints with `time_from_start` and
exposes the action MoveIt drives. See
[Step 8: Sim time and trajectories](docs/learning/step-08-sim-time-and-trajectories.md).

## Step 7: The `ros2_control` stack

Seven bespoke Gazebo controller plugins were replaced by the standard control
stack. Three declarations, three jobs:

| Where | What it says |
|---|---|
| `<ros2_control>` in the URDF | The **contract** — which joints exist, what can be commanded and read |
| `<gazebo>` system plugin | The **runtime** providing those interfaces inside the simulator |
| `panda_controllers.yaml` | The **controllers** to make available, and their configuration |

Controllers are registered by the YAML but started by a spawner, so an empty
`ros2 control list_controllers` before spawning is expected rather than a fault.
Commanded angles now track to within 1e-13 rad, against roughly 0.02 rad for the
per-joint PID it replaced. See
[Step 7: ros2_control](docs/learning/step-07-ros2-control.md).

## Step 6: One-command bringup

The bridge became a YAML table of eight topic mappings, and the whole twin now
starts from a single command with no environment sourcing and no clicking play:

```text
ros2 launch twin_action_demo panda_bringup.launch.py
  → Gazebo environment set inside launch
  → Gazebo started with '-r' (already running)
  → Panda spawned
  → bridge.launch.py included
```

This step also added `MoveArm`, a seven-joint action carrying arrays so the shape
matches `sensor_msgs/JointState.position`. It is superseded by Step 7's control
stack and kept for reference. See
[Step 6: One-command bringup](docs/learning/step-06-panda-bringup.md).

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
- `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-controllers`, and
  `ros-jazzy-robot-state-publisher` — required from Step 7
- `ros-jazzy-moveit` — required from Step 9, for the planning configuration

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

### Panda (Steps 4–7)

One command, from a plain shell. Do **not** source the Gazebo environment first —
the launch file sets it, starts the simulator already running, spawns the robot,
and activates both controllers:

```bash
ros2 launch twin_action_demo panda_bringup.launch.py
```

Confirm the control stack came up:

```bash
ros2 control list_controllers
#   arm_controller           joint_trajectory_controller/...  active
#   joint_state_broadcaster  joint_state_broadcaster/...      active
ros2 control list_hardware_interfaces      # 7 position interfaces, all claimed
```

Confirm the simulator's clock is reaching ROS — everything runs on sim time, so a
`/clock` with no publisher would leave the whole stack waiting:

```bash
ros2 topic info /clock       # Publisher count: 1
```

Command the arm with a **timed trajectory**, and read the result back by joint
name:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
  joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4,
                panda_joint5, panda_joint6, panda_joint7],
  points: [{ positions: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5],
             time_from_start: {sec: 2, nanosec: 0} }]
}"
ros2 topic echo /joint_states --once
```

The joint ordering is defined by the `joints:` list in
`twin_description/config/panda_controllers.yaml`. The same controller also
advertises `/arm_controller/follow_joint_trajectory`, which is how MoveIt will
execute plans.

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
  0.03 rad, with no drift across repeated samples at rest;
- one-command bringup from a plain shell with `gz_env.sh` unsourced: all eight
  bridge mappings created, and a seven-joint `MoveArm` goal returned success while
  a three-angle goal was rejected before any motion;
- under `ros2_control`: both controllers report `active`, all seven position
  command interfaces are claimed, and a commanded array is tracked to between
  1.9e-19 and 7.1e-13 rad per joint with velocities at numerical zero;
- on simulation time: `/clock` has one publisher at ~999.9 Hz, the controller
  manager's 47 prior "No clock received" warnings are gone, and a timed
  trajectory reaches target to within 7.1e-13 rad;
- the MoveIt configuration loads through `MoveItConfigsBuilder` with all six
  parameter sets present, resolves group `panda_arm` over `panda_link0 ->
  panda_link8` with 35 disabled collision pairs, and its execution seam names the
  `arm_controller` that actually exists.

## Known limitations

**Exact tracking probably means kinematic, not dynamic.** Errors of 1e-13 rad are
not evidence of a better controller; they indicate the position command reaches
the joint more or less directly rather than through a force-producing loop.
Nothing has been tested against contact or payload, and this should be confirmed
before treating the simulation as physically faithful.

**Nothing plans.** A trajectory still has to be written out by hand, waypoint by
waypoint. The MoveIt configuration exists and loads, but `move_group` has never
been run against the simulator, so there is no inverse kinematics, no collision
checking, and no Cartesian goal. The seam between MoveIt and the controller is
verified by configuration rather than by a trajectory arriving.

**Trajectory interpolation is unverified.** A timed goal is accepted and reached,
but sampling the motion mid-flight failed: the probe written for it runs on
wall-clock while the stack runs on sim time. A gap in the testing, not a known
fault.

**Two components are superseded but still present.** The `ros_gz` bridge still
starts and maps topics that no longer exist under `ros2_control`, and the `MoveArm`
action server publishes to those same dead topics. Both are harmless, both are
dead, and neither is wired into the current pipeline.

**The Panda description is generated and then hand-edited.** It is produced by
expanding the upstream xacro, after which the world anchor and the `ros2_control`
blocks are added by hand; re-running `xacro` discards them. The file header records
the command and the hazard.

**The Panda's mesh paths are absolute** into `/opt/ros/jazzy/...`, so the
description assumes a Jazzy install at the default prefix rather than being
portable across machines. The controllers-YAML path is *not* affected: it is
substituted by CMake at configure time from `panda.urdf.in`.

**The gripper is outside both layers.** Only the seven arm joints appear in
`<ros2_control>`, so the fingers have no interfaces and are not reported by the
broadcaster; and no end effector is configured in the SRDF, so they cannot be
planned for either. Their URDF mimic constraint is also unsupported by the DART
physics engine.

**Any new node must opt into simulation time.** Now that the clock is
authoritative, a node left on wall-clock desynchronises — as this project's own
test probe demonstrated.

**Re-running the MoveIt Setup Assistant may overwrite hand edits** to generated
files, including the controller rename that makes execution work at all. The
shipped `demo.launch.py` must not be used against this project: it stands up fake
hardware and a competing controller manager.

**Nothing enforces joint limits** before a command is published to the controller,
and self-collision is off — the Gazebo default, and intentional, since inter-link
collision belongs at the planning layer that arrives with MoveIt.

**Cancellation is reachable but untested.** Step 3 replaced the single-threaded
executor with a `MultiThreadedExecutor` and a `ReentrantCallbackGroup`, so a cancel
request can be serviced while a goal is executing — but no automated test covers
it.

## Repository layout

```text
.
├── docs/
│   ├── PROJECT-SPEC.md
│   └── learning/            one entry per milestone, step-01 … step-05
└── src/
    ├── twin_interfaces/     MoveJoint and MoveArm action definitions
    ├── twin_action_demo/    action servers, client, launch files, bridge config
    ├── twin_description/    URDF models and the controllers YAML
    └── twin_moveit_config/  SRDF, kinematics, and the MoveIt execution seam
```

`twin_description` holds data and stays launch-free; everything that starts a
process lives in `twin_action_demo`. The Panda description is committed as
`panda.urdf.in` because CMake substitutes the controllers-YAML path into it at
configure time — see [Step 7](docs/learning/step-07-ros2-control.md).

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
