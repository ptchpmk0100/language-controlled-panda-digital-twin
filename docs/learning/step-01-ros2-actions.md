# Step 1 — Build the ROS 2 Action Client/Server Pattern

## Objective

Learn the ROS 2 action lifecycle on the smallest useful system before
connecting the same communication pattern to Gazebo, ros2_control, and
MoveIt. The exercise uses a persistent numerical joint so goal validation,
feedback, results, and client/server responsibilities remain visible without
introducing simulator complexity.

This is a foundation exercise, not an end-to-end manipulation result.

## Outcome

The session produced two ROS 2 packages:

| Package | Build type | Responsibility |
|---|---|---|
| `twin_interfaces` | `ament_cmake` | Generate the `MoveJoint` action type |
| `twin_action_demo` | `ament_python` | Run the simulated server and async client |

The completed path is:

```text
parameterized target
        │
        ▼
asynchronous client
        │ MoveJoint goal
        ▼
simulated action server
        ├── reject target outside ±3.14 rad
        ├── update persistent numerical joint state
        ├── publish current angle and remaining error
        └── return success and final angle
```

## Session progression

1. Selected actions for operations that take time and need progress/results.
2. Created a dedicated `ament_cmake` interface package.
3. Corrected the node package from `ament_cmake` to `ament_python`.
4. Defined the three-section `MoveJoint.action` contract.
5. Added ROSIDL generation and runtime dependencies.
6. Diagnosed a missing `rosidl_default_generators` lookup.
7. Implemented a stateful simulated action server.
8. Registered the server as a Python console entry point.
9. Exercised accepted and rejected goals with the ROS CLI.
10. Inspected the action graph and hidden endpoints.
11. Implemented a one-shot asynchronous Python client.
12. Diagnosed a misspelled `target_angel` parameter override.
13. Verified success feedback/results and out-of-range rejection.

## Files and what they teach

### `src/twin_interfaces/action/MoveJoint.action`

Defines the action contract:

```text
# Goal
float64 target_angle
---
# Result
bool success
float64 final_angle
---
# Feedback
float64 current_angle
float64 remaining
```

The `---` separators divide goal, result, and feedback. ROSIDL generates the
Python and C/C++ types consumed by action clients and servers.

### `src/twin_interfaces/CMakeLists.txt`

The generator must be discovered before its CMake function is called:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJoint.action"
)
```

The session exposed an important build-system lesson: CMake cannot invoke
`rosidl_generate_interfaces` until the package that defines it has been found.

### `src/twin_interfaces/package.xml`

Declares build-time generation, runtime type support, and membership in the
ROS interface-package group. During portfolio validation, the dependency
elements were also placed in package-format order so schema validation passes.

### `src/twin_action_demo/twin_action_demo/move_joint_server.py`

Owns the server-side lifecycle:

- `goal_callback` accepts or rejects the requested target;
- `execute_callback` advances persistent joint state and publishes feedback;
- `cancel_callback` declares cancellation intent;
- `goal_handle.succeed()` establishes the terminal success state;
- the returned `MoveJoint.Result` carries the application result.

The joint starts at `0.0` and moves in `0.05 rad` increments. State persists
between goals, matching the idea that physical state does not reset when a
client exits.

### `src/twin_action_demo/twin_action_demo/move_joint_client.py`

Uses the asynchronous `rclpy` pattern:

```text
send_goal_async()
  └── goal_response_callback()
        └── get_result_async()
              └── result_callback()

feedback_callback() runs while execution progresses
```

The client spins until its `done` flag is set. This is suitable for a one-shot
test executable; a future command-executor node will remain active and submit
multiple actions.

### Python package scaffolding

`setup.py` maps ROS executable names to Python `main` functions:

```python
'move_joint_server = twin_action_demo.move_joint_server:main',
'move_joint_client = twin_action_demo.move_joint_client:main',
```

`setup.cfg`, the package resource marker, `__init__.py`, and package metadata
complete the `ament_python` layout.

## Debugging lessons

### Saved files are the build input

Colcon reads files from disk. An unsaved editor buffer does not change what
CMake or Python packaging sees.

### Package type determines its structure

The interface package uses `ament_cmake`, while the Python node package uses
`ament_python`. The latter has `setup.py`, a resource marker, and a nested
Python module instead of a CMake source tree.

### Verify generated interfaces directly

After building and sourcing:

```bash
ros2 interface show twin_interfaces/action/MoveJoint
```

This isolates interface-generation problems before server/client debugging.

### Parameter overrides require exact names

The intended command is:

```bash
ros2 run twin_action_demo move_joint_client \
  --ros-args -p target_angle:=1.0
```

A misspelled new parameter may be ignored while the declared
`target_angle` retains its default. The client's “Preparing target angle”
line provides a simple observability check.

### Read both sides of a distributed interaction

Client logs show request, acceptance/rejection, feedback, and result handling.
Server logs show validation and execution. Neither side alone proves the
whole path.

## Technical corrections retained in the public record

A ROS 2 action is implemented with **three services and two topics**:

```text
/<action>/_action/send_goal
/<action>/_action/get_result
/<action>/_action/cancel_goal
/<action>/_action/feedback
/<action>/_action/status
```

These endpoints contain the hidden `_action` token. Use CLI options that
include hidden topics/services when inspecting the raw graph. Feedback
messages are produced during execution, but omission from a default
post-execution topic listing does not prove that the endpoint entity was
created only for that goal.

## Verification

The reconstructed milestone was validated on Ubuntu 24.04 with ROS 2 Jazzy:

```text
colcon build
  Summary: 2 packages finished

twin_action_demo tests
  2 passed, 1 skipped (generated copyright check)

twin_interfaces tests
  CMake lint passed
  package.xml schema validation passed
```

Runtime checks:

| Case | Target | Observed outcome |
|---|---:|---|
| Accepted | `1.0 rad` | Feedback from `0.05` to `0.95`; success at `1.00` |
| Rejected | `5.0 rad` | Goal rejected before simulated motion |
| Cancel audit | long-running goal | Cancel request was not processed until execution ended |

The cancel audit is intentionally recorded as a limitation. The callback is
present, but the synchronous single-threaded execution design prevents prompt
cancel servicing. This milestone therefore demonstrates the action contract
and nominal/rejection paths, not reliable cancellation.

## Commit boundaries

The capability-based history for this milestone is:

1. `docs: define digital twin scope and acceptance criteria`
2. `feat(interfaces): define single-joint motion action`
3. `feat(actions): implement simulated single-joint action server`
4. `feat(actions): add asynchronous MoveJoint client`
5. `docs: document the ROS 2 action learning milestone`

These boundaries represent complete ideas. They are intentionally not padded
into equal daily commit counts.

## Next engineering step

Replace the numerical update loop with a controller-backed joint command and
measured joint-state feedback. The action contract and client flow can remain
stable while the server implementation moves from fake state to Gazebo and,
later, MoveIt-planned trajectories.

