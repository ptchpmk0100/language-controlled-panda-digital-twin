# Language-Controlled Panda Digital Twin

An incremental ROS 2 robotics project whose end goal is a simulation-only
Franka Panda manipulator that executes validated natural-language
pick-and-place commands in Gazebo.

> **Current status:** foundation prototype. The repository demonstrates a custom
> ROS 2 action on a simulated numerical joint, and a physically simulated
> one-joint arm whose real joint state is bridged from Gazebo into ROS 2. It
> does not yet contain the Panda model, MoveIt planning, perception, or
> language-model integration described in the project specification.

## Milestones

| Step | Capability | Documentation |
|---|---|---|
| 1 | Custom ROS 2 action on a simulated numerical joint | [Step 1](docs/learning/step-01-ros2-actions.md) |
| 2 | Gazebo physics + real joint state bridged into ROS 2 | [Step 2](docs/learning/step-02-gazebo-pipeline.md) |

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

Nothing drives the joint yet — the arm falls and settles near π/2, which is correct
for an unpowered link. The MoveIt Panda description was tried first and rejected by
Gazebo for having no `<inertial>` blocks; that failure is what motivated building a
minimal arm to prove the pipeline. See
[Step 2: Gazebo pipeline](docs/learning/step-02-gazebo-pipeline.md).

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

Start the server:

```bash
ros2 run twin_action_demo move_joint_server
```

In a second sourced terminal, send an accepted goal:

```bash
ros2 run twin_action_demo move_joint_client \
  --ros-args -p target_angle:=1.0
```

Exercise the bounds check:

```bash
ros2 run twin_action_demo move_joint_client \
  --ros-args -p target_angle:=5.0
```

The accepted goal should stream feedback and finish at `1.00 rad`. The
out-of-range goal should be rejected before simulated motion begins.

### Gazebo joint-state pipeline (Step 2)

With the Gazebo environment exported, start a simulator and spawn the arm:

```bash
gz sim empty.sdf          # press play; the plugin only publishes while stepping
ros2 run ros_gz_sim create -world empty -name one_joint_arm \
  -file "$(ros2 pkg prefix twin_description)/share/twin_description/urdf/one_joint_arm.urdf"
```

Confirm Gazebo is publishing before involving the bridge, then relay to ROS 2:

```bash
gz topic -l | grep joint
ros2 run ros_gz_bridge parameter_bridge \
  "/world/empty/model/one_joint_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
ros2 topic echo /world/empty/model/one_joint_arm/joint_state
```

The arm swings down and settles near `1.57 rad`. Nothing commands it — that is
gravity, and it is the expected result for a robot with no controller.

## Test

```bash
colcon test
colcon test-result --verbose
```

Validated on ROS 2 Jazzy (11 tests, 0 failures, 1 skipped):

- clean build of all three packages;
- CMake and package XML checks passed;
- Python Flake8 and PEP 257 checks passed;
- accepted goal streamed feedback and returned success;
- out-of-range goal was rejected;
- `one_joint_arm.urdf` parsed and spawned; bridged `sensor_msgs/JointState`
  observed at `position ≈ 1.576 rad`, `velocity ≈ 0.0017`.

## Known limitations

The learning server uses a synchronous execution callback with a
single-threaded executor. Although it registers a cancel callback and checks
`is_cancel_requested`, a cancel request cannot be serviced while that
callback is blocking. Reliable concurrent cancellation is therefore not
claimed for this milestone. A later controller-backed implementation must use
an executor/callback-group design that can service state and cancellation
callbacks during motion.

The simulated arm cannot yet be commanded. Step 2 bridges joint state in one
direction only (Gazebo → ROS 2); no controller drives the joint, so the action
server still advances a number rather than the simulated robot.

## Repository layout

```text
.
├── docs/
│   ├── PROJECT-SPEC.md
│   └── learning/
└── src/
    ├── twin_interfaces/
    ├── twin_action_demo/
    └── twin_description/
```

## Development

This project is built as a guided learning exercise, with AI pair-programming
(Claude) used for teaching, review, and documentation. The engineering decisions,
debugging, and verification recorded in `docs/learning/` are the substance of the
work; each milestone is built and run before it is documented.

Generated `build/`, `install/`, and `log/` trees are intentionally excluded
from version control.

