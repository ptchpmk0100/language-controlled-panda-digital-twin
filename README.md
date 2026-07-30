# Language-Controlled Panda Digital Twin

An incremental ROS 2 robotics project whose end goal is a simulation-only
Franka Panda manipulator that executes validated natural-language
pick-and-place commands in Gazebo.

> **Current status:** foundation prototype. The repository currently
> demonstrates a custom ROS 2 action on a simulated numerical joint. It does
> not yet contain the Panda simulation, MoveIt planning, perception, or
> language-model integration described in the project specification.

## Current milestone: ROS 2 action lifecycle

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

## Test

```bash
colcon test
colcon test-result --verbose
```

Validated on ROS 2 Jazzy:

- clean build of both packages;
- CMake and package XML checks passed;
- Python Flake8 and PEP 257 checks passed;
- accepted goal streamed feedback and returned success;
- out-of-range goal was rejected.

## Known limitation

The learning server uses a synchronous execution callback with a
single-threaded executor. Although it registers a cancel callback and checks
`is_cancel_requested`, a cancel request cannot be serviced while that
callback is blocking. Reliable concurrent cancellation is therefore not
claimed for this milestone. A later controller-backed implementation must use
an executor/callback-group design that can service state and cancellation
callbacks during motion.

## Repository layout

```text
.
├── docs/
│   ├── PROJECT-SPEC.md
│   └── learning/
└── src/
    ├── twin_interfaces/
    └── twin_action_demo/
```

Generated `build/`, `install/`, and `log/` trees are intentionally excluded
from version control.

