# Step 6 — One Command, Seven Joints

## Objective

Make the Panda usable. Step 5 left seven working controllers that could only be
reached over gz transport, with a hand-typed bridge and a manual click of play.
This milestone closes that gap in three pieces: a declarative seven-way bridge, a
single launch command that brings the whole twin up, and a seven-joint action that
finishes what Step 3 started for one joint.

## Outcome

```text
ros2 launch twin_action_demo panda_bringup.launch.py
        │
        ├── Gazebo environment set inside launch  (no gz_env.sh)
        ├── Gazebo started with '-r'              (no clicking play)
        ├── Panda spawned
        └── bridge.launch.py included             (8 topic mappings)

ros2 run twin_action_demo move_arm_server
        │
        ▼
MoveArm goal: 7 target angles
        │  publishes to 7 cmd_pos topics
        ▼
7 × JointPositionController
        │  measured back on one joint_state topic
        ▼
array feedback, and success when the SLOWEST joint arrives
```

## The bridge as a table

Step 3's bridge was two inline argument strings. Seven joints make that shape
untenable, so the mappings move into a YAML file that the bridge reads as a
parameter:

```yaml
- ros_topic_name: "/panda_joint1/cmd_pos"
  gz_topic_name: "/panda_joint1/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
```

`direction` is the bracket, spelled out: `ROS_TO_GZ` is `]`, `GZ_TO_ROS` is `[`.
Eight entries — one measurement coming out, seven commands going in.

Two details are easy to get wrong:

- The config arrives as `parameters=[{'config_file': ...}]`, **not** as
  `arguments=[...]`. Same node, different mechanism.
- An `ament_python` package installs no data files by default. Without an
  explicit `config` entry in `setup.py`, the launch file resolves a share path
  that does not exist.

The bridge lives in its own launch file so it stays independently runnable. When
commands are not reaching the robot, launching only the bridge separates "is the
bridge wired?" from everything else.

## Baking the environment into launch

Every previous step began with `source ~/gz_env.sh` in the simulator's terminal.
Two ways to remove that ritual: keep sourcing and launch afterwards, or set the
variables inside the launch file. This project took the second, so the bringup
works from a plain shell.

The launch actions reproduce the same globs `gz_env.sh` uses:

```python
GZ_CONFIG_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))
...
set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATHS)
append_path = AppendEnvironmentVariable('PATH', GZ_TOOLS_BIN)
```

**`Set` overwrites; `Append` preserves.** `GZ_VERSION` and `GZ_CONFIG_PATH` are
ours to define. `PATH` and `LD_LIBRARY_PATH` belong to the caller, and
overwriting either would break the very shell the launch runs in.

Gazebo itself is pulled in with `IncludeLaunchDescription`, passing
`gz_args: 'empty.sdf -r'`. The `-r` starts the world running, which is what
replaces the manual press of play — and without it, the PID loops never step and
every command looks ignored.

Composition rather than duplication is the theme: this file includes
`ros_gz_sim`'s launch file *and* the project's own `bridge.launch.py`.

## The seven-joint action

`MoveArm` is `MoveJoint` widened from a scalar to an array:

```text
float64[] target_angles
---
bool success
float64[] final_angles
---
float64[] current_angles
float64[] remaining
```

Arrays rather than seven named fields: the shape then matches
`sensor_msgs/JointState.position`, and changing the joint count becomes
configuration rather than an interface change. The cost is that length and
ordering become the caller's problem, so the server validates both and rejects a
goal that is not exactly seven angles.

`MoveJoint` and the Step 3 nodes are left intact. They still target the
one-joint arm's topics and are not part of the Panda pipeline.

Three things in the server are worth naming:

**Look joints up by name, never by index.** The `joint_state` message also
carries `world_to_panda` and the two finger joints, and its ordering is not
guaranteed. `msg.name.index(joint)` is the only safe read.

**"Arrived" means the slowest joint arrived** — `max(errors) <= tolerance`. Any
other reduction would report success while something was still moving.

**The concurrency design is unchanged from Step 3**, and it has to be. The
execute callback polls values that a subscription supplies, so the two need the
`ReentrantCallbackGroup` and `MultiThreadedExecutor` pairing. Reverting to a
single-threaded executor deadlocks in exactly the way Step 3 documented.

## Correction made while committing this milestone

The session's `move_arm_server` waits for its first measurement in an unbounded
loop. If the bridge is down or the simulator is paused, the goal blocks forever
with no diagnosis. Bounded, as Step 3's server already was, and the abort message
now names which joints were never measured. Same change, same reason.

## Verification

Run deliberately from a **plain shell with `gz_env.sh` not sourced**, since that
is the claim under test:

```text
GZ_VERSION=''      (before launch)
```

All eight mappings, from the bringup log:

```text
Creating GZ->ROS Bridge: [/world/empty/model/panda/joint_state (gz.msgs.Model)
                       -> ... (sensor_msgs/msg/JointState)]
Creating ROS->GZ Bridge: [/panda_joint1/cmd_pos (std_msgs/msg/Float64)
                       -> ... (gz.msgs.Double)]
... joints 2 through 7 ...
```

The generated interface:

```text
ros2 interface show twin_interfaces/action/MoveArm
  float64[] target_angles
  ---
  bool success
  float64[] final_angles
  ---
  float64[] current_angles
  float64[] remaining
```

A seven-joint goal of `[0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]`:

```text
Goal accepted with ID: e2f0baa7b34442da87c403e66a47d827
Result:
    success: true
```

Measured back through the bridge afterwards:

| Joint | Target | Measured | Error |
|---|---|---|---|
| `panda_joint1` | +0.00 | −0.0003 | 0.0003 |
| `panda_joint2` | −0.50 | −0.4918 | 0.0082 |
| `panda_joint3` | +0.00 | +0.0005 | 0.0005 |
| `panda_joint4` | −1.50 | −1.5197 | 0.0197 |
| `panda_joint5` | +0.00 | −0.0027 | 0.0027 |
| `panda_joint6` | +1.00 | +0.9980 | 0.0020 |
| `panda_joint7` | +0.50 | +0.4385 | 0.0615 |

`panda_joint7` reads outside the 0.05 rad tolerance here because the sample was
taken three seconds after the result was returned; success is judged at the
moment every joint is simultaneously inside the band, and this wrist joint drifts
afterwards — the same weakness Step 5 measured.

Validation rejects a malformed goal before any motion:

```text
ros2 action send_goal /move_arm ... "{target_angles: [0.0, 0.0, 0.0]}"
  Goal was rejected.
```

Build and lint: 11 tests, 0 errors, 0 failures, 1 skipped.

## Lessons

**A configuration format is a readability decision.** Eight inline bridge
specifications would parse identically and be unreviewable. A table has one row
per topic and a column per field, and a wrong entry is visible.

**Environment is part of the deliverable.** "Source this first" is a step that
can be forgotten, done in the wrong terminal, or lost when someone else clones
the repository. Moving it into launch makes the bringup reproducible.

**Widening an interface is where sloppy assumptions surface.** Going from one
joint to seven turned "read the position" into "read the position of a named
joint from a message that also contains joints you do not control, in an order
nobody promised."

## Known limitations

- **Two commands, not one.** The bringup does not start `move_arm_server`;
  it is run separately.
- **No coordination between joints.** Seven independent setpoints are published
  at once, and each joint runs its own PID to get there. Nothing produces a timed
  trajectory, so the path between poses is whatever the controllers happen to do.
- **The wrist joints still drift** after the goal reports success, so
  `final_angles` is a snapshot rather than a settled measurement.
- **A rejected goal cannot say why** beyond the server's log line; the result
  message carries no reason field.
- **The 15-second timeout is fixed** and not exposed as a parameter. When a goal
  does time out, the last feedback's `remaining` array identifies the joint that
  failed to converge.
- **Gripper joints are not commanded**, and `MoveJoint` and the Step 3 nodes
  remain wired to the one-joint arm.

## Commit boundaries

1. `feat(bringup): bridge all Panda topics from a YAML config`
2. `feat(bringup): start the whole twin from one launch command`
3. `feat(actions): add a seven-joint MoveArm action and server`
4. `docs: document the one-command bringup milestone`

## Next engineering step

Move to `ros2_control`. Seven per-joint plugins are simple and proven, but the
gains are trapped in the description, nothing produces trajectories, and MoveIt —
the reason this project needs planning at all — expects `ros2_control` interfaces
rather than raw command topics.
