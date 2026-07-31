# Step 7 — Rearchitect onto `ros2_control`

## Objective

Replace seven independent Gazebo controller plugins with the standard ROS 2
control stack. This is not a feature; it is the interface MoveIt expects, and it
moves controller configuration out of the robot description and into YAML.

It is also the first step in this project that *deletes* working code. Seven
proven `JointPositionController` blocks and the `JointStatePublisher` are removed
in favour of a contract and a single system plugin.

## Three jobs, three places

The confusing part of `ros2_control` is that a working setup needs three separate
declarations, and it is easy to think one of them should be enough.

| Where | What it says |
|---|---|
| `<ros2_control>` in the URDF | The **contract** — which joints exist, what can be commanded, what can be read |
| `<gazebo>` system plugin | The **runtime** that provides those interfaces inside the simulator |
| `panda_controllers.yaml` | The **controllers** to make available, and their configuration |

The contract is plain URDF, not a Gazebo plugin. It declares intent:

```xml
<ros2_control name="PandaSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="panda_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

The runtime is one `<gazebo>` block, and this is where the two plugin strings
trip people up. They look interchangeable and are not:

- `gz_ros2_control/GazeboSimSystem` — the hardware-interface **class** the
  controller manager loads, named in `<hardware>` above.
- `gz_ros2_control-system` / `gz_ros2_control::GazeboSimROS2ControlPlugin` — the
  Gazebo **system library** that starts a controller manager inside the sim.

Both are required, in different places.

## Controllers do not start themselves

The YAML *registers* controllers by name and type. It does not run them:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_position_controller:
      type: position_controllers/JointGroupPositionController
```

A **spawner** loads and activates each one. An empty `ros2 control
list_controllers` before spawning is the expected state, not a fault — which is
worth knowing, because it looks exactly like a broken configuration.

No PID gains appear anywhere. `JointGroupPositionController` is a *forward*
controller: it passes an array of positions to the position command interface and
the hardware closes the loop. Step 5's `p=800, i=20, d=60` was not ported because
nothing along this path runs a PID any more. Gains return only on an effort or
velocity interface.

## The failure that cost the session

`gz_ros2_control` reads the robot description from the **`/robot_description`
topic**, not from the file it was spawned with. With nothing publishing that
topic, the controller manager waits:

```text
Waiting for data on 'robot_description'
```

forever. No error, no exit — just a hang, and spawners that fail because no
manager ever appears. The fix is to add `robot_state_publisher` to the bringup,
which publishes the URDF on that topic:

```python
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(
            Command(['cat ', urdf_path]), value_type=str
        ),
    }],
)
```

`cat` because this is flat URDF; an xacro source would substitute `xacro` here.

## Sequencing instead of racing

Spawners cannot run before the manager exists, and the manager only exists once
the robot has been spawned into a running simulator. Rather than guessing at
delays, the launch file chains on process exit:

```python
RegisterEventHandler(OnProcessExit(target_action=spawn,
                                   on_exit=[joint_state_broadcaster_spawner]))
RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                   on_exit=[position_controller_spawner]))
```

Spawn finishes, then the broadcaster, then the position controller.

## Correction made while committing this milestone

`<parameters>` is read as a **literal filesystem path**. The URDF is handed to
Gazebo as raw file contents, so nothing expands `package://` or `$(find ...)`
inside it, and a launch substitution would arrive at the plugin verbatim. The
session's solution was to hard-code an absolute path into the author's home
directory, and to log the non-portability as a loose end.

That cannot be committed: a repository that only works for one user on one
machine is not a repository. The description is therefore committed as
`urdf/panda.urdf.in`, with CMake substituting the path at configure time:

```cmake
set(TWIN_CONTROLLERS_YAML
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/panda_controllers.yaml")
configure_file(urdf/panda.urdf.in "${CMAKE_CURRENT_BINARY_DIR}/urdf/panda.urdf" @ONLY)
```

A fresh clone then points at its own install tree instead of at the workspace it
was copied from. The template stays the source of truth, the usable URDF is
installed from the build tree, and the `.in` file is excluded from the install.

This is the same class of problem as Step 3's hard-coded launch path, solved at a
different layer because the launch system never gets to see inside this file.

## Verification

One command, from a plain shell:

```text
ros2 launch twin_action_demo panda_bringup.launch.py

[spawner_joint_state_broadcaster]:   Configured and activated joint_state_broadcaster
[spawner_joint_position_controller]: Configured and activated joint_position_controller
```

```text
ros2 control list_controllers
  joint_position_controller  position_controllers/JointGroupPositionController  active
  joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster      active
```

Every declared command interface is claimed by a controller:

```text
ros2 control list_hardware_interfaces
  command interfaces
      panda_joint1/position [available] [claimed]
      ... through panda_joint7 ...
  state interfaces
      panda_joint1/position
      panda_joint1/velocity
      ...
```

Commanding through the new path, and reading `/joint_states` back by name:

```bash
ros2 topic pub --once /joint_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]}"
```

| Joint | Target | Measured | Error | Velocity |
|---|---|---|---|---|
| `panda_joint1` | +0.00 | +0.000000000000000 | 1.9e-19 | 1.3e-18 |
| `panda_joint2` | −0.50 | −0.499999999999912 | 8.8e-14 | −2.4e-14 |
| `panda_joint3` | +0.00 | +0.000000000000010 | 1.0e-14 | −2.8e-18 |
| `panda_joint4` | −1.50 | −1.500000000000709 | 7.1e-13 | −8.5e-14 |
| `panda_joint5` | +0.00 | −0.000000000000158 | 1.6e-13 | 5.4e-17 |
| `panda_joint6` | +1.00 | +0.999999999999407 | 5.9e-13 | 2.9e-14 |
| `panda_joint7` | +0.50 | +0.500000000000000 | 2.2e-16 | 2.3e-14 |

Step 5's per-joint PID held these same joints to roughly 0.02 rad, with visible
droop on the wrists. This is around eleven orders of magnitude tighter, with
velocities at numerical zero. The droop and setpoint-hunting loose ends carried
since Step 3 are gone.

That result deserves a caveat rather than a victory lap — see the first known
limitation.

Also visible, and expected:

```text
[gz_ros_control]: Fixed joint ['world_to_panda'] (Entity='43') is skipped.
```

The world anchor is a fixed joint with no interfaces to claim.

Build and lint: 11 tests, 0 errors, 0 failures, 1 skipped.

## Lessons

**A hang is a worse failure mode than an error.** "Waiting for data on
`robot_description`" is a correct, informative message that produces no exit code
and no stack trace, and the visible symptom is downstream: spawners failing for
no apparent reason. Read the log of the process that is quiet, not only the one
that is complaining.

**Sequence explicitly rather than hoping.** Event handlers express "after this
exits" directly. Sleeps encode a guess about a machine's speed.

**Deleting working code is part of the job.** Seven proven plugins were removed.
Keeping them alongside the new stack would have produced two systems fighting
over the same joints.

**A configuration mechanism that reads raw text cannot be given a
substitution.** Knowing *when* each layer resolves paths — launch at start-up,
CMake at configure time, the plugin never — is what makes the fix obvious rather
than a guess.

## Known limitations

- **Exact tracking probably means kinematic, not dynamic.** Errors of 1e-13 rad
  are not a better controller; they indicate the position command is being
  applied to the joint more or less directly rather than through a
  force-producing loop. Nothing here has been tested against contact or payload,
  and this is worth confirming before treating the twin as physically faithful.
- **`use_sim_time` is not set**, so the controller manager logs
  `No clock received, using time argument instead!` — 47 times during this run.
  Benign for position commands, which carry no timestamps, but it must be fixed
  before MoveIt, whose trajectories are timed.
- **The `ros_gz` bridge is now vestigial.** It still starts and still creates
  eight mappings for `/panda_jointN/cmd_pos` and the gz `joint_state`, none of
  which exist any more. Harmless, dead, and left in place; removing it cleanly is
  its own small task.
- **`MoveArm` and its server are superseded.** They publish to `cmd_pos` and
  subscribe to the old gz `joint_state`, so neither is wired into this pipeline.
  Both are kept on disk for reference and should not be relied on.
- **Commands are still untimed positions.** `Float64MultiArray` says where to go,
  not how or how fast. MoveIt needs a `JointTrajectoryController`, which does not
  exist yet.
- **Mesh paths remain absolute** into `/opt/ros/jazzy/...`, and the description is
  still generated then hand-edited.
- **The gripper is still outside the contract**: only the seven arm joints appear
  in `<ros2_control>`, so the fingers have no interfaces and are not reported by
  the broadcaster.

## Commit boundaries

1. `feat(description): move the Panda onto ros2_control`
2. `feat(bringup): publish robot_description and auto-spawn controllers`
3. `docs: document the ros2_control rearchitecture`

## Next engineering step

Set `use_sim_time` so the controller manager follows Gazebo's clock, then add a
`JointTrajectoryController` — the interface MoveIt drives — and build the MoveIt
configuration for real `move_to(x, y, z)` planning.
