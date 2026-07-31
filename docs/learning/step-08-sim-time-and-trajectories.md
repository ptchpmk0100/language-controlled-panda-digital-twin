# Step 8 — Sim Time, and a Controller That Accepts Trajectories

## Objective

Clear the two things standing between the control stack and MoveIt.

MoveIt sends *timed* trajectories and stamps everything against the clock. Step 7
left a controller that takes untimed position arrays, and a controller manager
that was not following the simulator's clock at all. Neither is a feature gap;
both are interface mismatches that would surface as unexplained execution
failures later.

## The clock, and the hidden cause underneath it

Step 7 logged this warning forty-seven times in a single run:

```text
[controller_manager]: No clock received, using time argument instead!
Check your node's clock configuration (use_sim_time parameter)
```

The obvious fix is one line — `use_sim_time: true` in the controller manager's
parameters. Applying it exposed a second problem the warning had been masking:

```text
ros2 topic info /clock
  Publisher count: 0
```

**Gazebo does not publish `/clock` to ROS.** It keeps simulation time internally,
and a bridge has to carry it across. Nothing had ever bridged it, because until
now nothing had asked for it. Setting `use_sim_time` without that bridge is
strictly worse than leaving it alone: every node then waits for a clock that
never arrives.

```python
clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
)
```

The `[` is the gz-to-ROS direction — the simulator owns time and ROS consumes it.

**`use_sim_time` is per node and defaults to false.** Being hosted *inside*
Gazebo does not make the controller manager infer sim-time; it has to be told.
So does `robot_state_publisher`, separately, because a split clock — one node on
sim-time, the other on wall-clock — desynchronises TF.

The clock bridge is deliberately kept out of `bridge.launch.py`. That file is
vestigial under `ros2_control` and will be deleted; putting something load-bearing
inside it would turn a clean delete into a trap.

## Replacing the controller

`JointGroupPositionController` takes an array of positions and passes it to the
command interface. There is no notion of time, so there is nothing to interpolate
between — the target is simply the new setpoint.

`JointTrajectoryController` takes a `trajectory_msgs/JointTrajectory`: joint
names, and waypoints each carrying a `time_from_start`. It interpolates between
them on every control cycle, and it exposes a `FollowJointTrajectory` action —
which is exactly what MoveIt drives.

The swap is a replacement, not an addition. `ros2_control` enforces exclusive
ownership of a joint's command interface, so two controllers cannot both hold the
same seven joints.

```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
```

Two details worth carrying:

- **The type namespace changes.** The package is `joint_trajectory_controller`,
  not `position_controllers`.
- **The name is `arm_controller`, not something descriptive of its type.** MoveIt
  refers to controllers by name, and this is the string its configuration will
  have to match. Naming it now avoids a rename later at the one seam where a
  mismatch fails silently.

The trajectory controller also needs its `command_interfaces` and
`state_interfaces` declared, which the group controller did not: it reads state
to track its progress along a trajectory, not merely to report it.

## Rename discipline

The old name has to reach zero live references — the YAML registry, the spawner,
the event handler, and the entry in the returned launch description. Half a
rename produces a launch file that references a controller nobody spawns, or a
spawner waiting on a name the manager never registered.

```bash
grep -rn 'joint_position_controller\|JointGroupPositionController' src/   # empty
```

## Verification

Bringup, then measurement:

```text
"No clock received" occurrences in the launch log:  0     (was 47)

ros2 topic info /clock
  Type: rosgraph_msgs/msg/Clock
  Publisher count: 1
  Subscription count: 1

ros2 topic hz /clock
  average rate: 999.902   min 0.000s  max 0.002s  std dev 0.00016s

ros2 control list_controllers
  arm_controller          joint_trajectory_controller/JointTrajectoryController  active
  joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active

ros2 action list
  /arm_controller/follow_joint_trajectory
```

That last line is the point of the whole step: it is the action MoveIt executes
through.

A timed trajectory to `/arm_controller/joint_trajectory` with
`time_from_start: 2s`, then `/joint_states` read back by name:

| Joint | Target | Measured | Error |
|---|---|---|---|
| `panda_joint1` | +0.00 | +0.000000000000000 | 1.8e-19 |
| `panda_joint2` | −0.50 | −0.499999999999912 | 8.8e-14 |
| `panda_joint3` | +0.00 | +0.000000000000010 | 1.0e-14 |
| `panda_joint4` | −1.50 | −1.500000000000708 | 7.1e-13 |
| `panda_joint5` | +0.00 | −0.000000000000160 | 1.6e-13 |
| `panda_joint6` | +1.00 | +0.999999999999408 | 5.9e-13 |
| `panda_joint7` | +0.50 | +0.500000000000000 | 2.2e-16 |

The old command path is gone rather than deprecated:

```text
ros2 topic list | grep -c joint_position_controller   →   0
```

Build and lint: 11 tests, 0 errors, 0 failures, 1 skipped.

### What was not verified

**The smooth ramp was not captured.** The distinguishing behaviour of a
trajectory controller is that it interpolates rather than jumping, and this
re-run did not demonstrate it. The trajectory *interface* is verified — a timed
goal is accepted and reached to 1e-13 — but sampling mid-motion failed: the probe
written for it runs on wall-clock while the rest of the stack runs on sim-time,
so its own timing is not trustworthy, and one run captured a motion that started
and then stalled at −0.0393 rad.

That is a gap in the verification, not a known fault in the controller. Sampling
interpolation properly needs a probe that also sets `use_sim_time`, which is the
same lesson this step is about.

## Lessons

**A warning can be hiding its own cause.** "No clock received" names the
`use_sim_time` parameter, which makes the parameter look like the whole problem.
Setting it correctly is what revealed that `/clock` had no publisher at all.

**Diagnose with a query, not a guess.** `ros2 topic info /clock` showing
`Publisher count: 0` turned an ambiguous warning into a specific missing
component.

**"Active" is not "works."** A controller reporting `active` has loaded and
claimed its joints. Whether it moves the robot is a separate question that only
sending it a real command answers.

**Test through the interface that will be used in anger.** The array-poke was a
convenient crutch; the trajectory topic is what MoveIt and real hardware use.
Validating through the crutch would have proven nothing about the path that
matters.

## Known limitations

- **Interpolation is unverified here** — see above.
- **`bridge.launch.py` is still vestigial** and still included in the bringup,
  mapping topics that no longer exist. The clock bridge is deliberately separate
  so that removing it stays a clean delete.
- **`MoveArm` and its server remain superseded**, publishing to the same dead
  topics.
- **Nothing plans.** A trajectory still has to be written out by hand, waypoint
  by waypoint. There is no IK, no collision checking, and no notion of a Cartesian
  goal — that is what MoveIt is for.
- **Any new node must opt into sim-time.** Now that the clock is authoritative,
  a node left on wall-clock will desynchronise, as this step's own probe
  demonstrated.
- **The gripper is still outside the contract**, and the description is still
  generated-then-hand-edited with absolute mesh paths.

## Commit boundaries

1. `feat(control): run on sim time and drive the arm with trajectories`
2. `docs: document the sim-time fix and the trajectory controller swap`

## Next engineering step

Build the MoveIt configuration — planning group, IK solver, joint limits, and the
self-collision matrix — and point its execution configuration at
`arm_controller`, the name chosen here for exactly that purpose.
