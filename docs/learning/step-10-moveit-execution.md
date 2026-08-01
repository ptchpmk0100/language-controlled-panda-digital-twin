# Step 10 — Plan in MoveIt, Move in Gazebo

## Objective

Connect the two halves. Step 9 built a planning configuration; Steps 7 and 8
built a controller that executes trajectories. This milestone runs `move_group`
against the live Gazebo stack and gets a planned motion to actually move the arm.

Nothing new is designed here. The work is entirely in clearing five failures
that stood between a correct-looking configuration and a robot that moves.

## Launching the planner alone

The Setup Assistant's `demo.launch.py` stands up fake hardware, its own
controller manager, `move_group`, and RViz together. The robot and its
controllers already exist, so what is needed is `move_group` by itself:

```python
move_group_node = Node(
    package='moveit_ros_move_group', executable='move_group',
    parameters=[
        moveit_config.to_dict(),
        move_group_configuration,
        {'use_sim_time': True},      # last dict wins
    ],
)
```

`generate_move_group_launch` offers no hook for `use_sim_time`, so the node is
assembled by hand. Parameter dictionaries merge last-wins, which is why it goes
at the end. This is the same clock seam as Step 8, at a new node.

The launch file deliberately does **not** start `robot_state_publisher` or spawn
controllers — the Gazebo bringup already owns both, and doing it twice produces
two publishers of `/robot_description` and a spawner fighting for joints that are
already claimed.

## Five failures, in the order they appeared

Each surfaced only after the previous one was fixed. This is normal for MoveIt
bring-up, and the sequence is the substance of the milestone.

### 1. A robot state that was never complete

`Missing panda_finger_joint1`, repeating once a second, and no planning at all.

`move_group` builds its model from the full URDF — nine joints. The joint state
broadcaster reports only the seven listed in `<ros2_control>`. The two fingers had
no state source anywhere, so the planning-scene monitor never assembled a complete
robot state and refused to plan.

The fix is a small node publishing only the two finger joints. What makes it safe
is ownership: the broadcaster owns the seven arm joints, this node owns the two
fingers, and no joint is published by both. Two publishers of disjoint joints
merge cleanly by name.

It *reports* finger state; it does not command the fingers.

### 2. A build that succeeded without the code

`ModuleNotFoundError: finger_state_publisher` — from a package that had just
built cleanly.

An `ament_python` console-script entry point installs a stub that imports its
module at *run* time. The stub builds fine whether or not the module exists, so a
file that never got written produces a successful build and a runtime failure.

The check is not "did the build succeed" but "is the module on disk":

```bash
ls install/twin_action_demo/lib/python3.12/site-packages/twin_action_demo/
```

### 3. A gap in the collision matrix

`CheckStartStateCollision failed: panda_link5 - panda_rightfinger`.

Step 9's matrix disabled `panda_link5` against links 3, 4, 6 and 7 — but not
against either finger. At the runtime finger position of 0.02 m those capsules
graze, so the robot was in self-collision *at its start state*, and a planner will
not plan out of a colliding state.

This is a sampling gap, not a mistake: the assistant samples poses, and this
combination did not come up. Two pairs added, both `reason="Never"`, which is
honest here — link 5 and a fingertip cannot meaningfully collide.

**The failure mode is what matters.** Disabling a pair is a safety statement. This
one was safe; a careless one tells the planner it may drive the arm through
itself.

### 4. A planner that requires what was not given

`planner ID '' does not exist`.

RViz defaulted to `pilz_industrial_motion_planner`, which requires an explicit
`PTP`, `LIN` or `CIRC` planner id. An empty id is fine for OMPL, which is a
general sampling planner, and fatal for Pilz.

This one is a UI setting rather than a file, and it does not persist — RViz
returns to Pilz on every restart.

### 5. Timing that could not be computed

`No acceleration limit was defined for joint panda_joint1`.

OMPL returns a *geometric* path — a sequence of configurations with no timing.
Turning that into an executable trajectory is a separate stage,
`AddTimeOptimalParameterization`, and it needs both velocity and acceleration
limits. The generated `joint_limits.yaml` had acceleration limits switched off.

Enabling them (15.0 for joints 1–4, 20.0 for 5–7) is what makes a plan
executable. `default_acceleration_scaling_factor: 0.1` still keeps the motion
gentle.

## A scope error worth recording

This acceleration-limit change belongs to this milestone, but it was **committed
one step early**, in Step 9. The file copied into the repository already carried
the fix, and it was not caught before committing. The Step 9 tag therefore
contains a change Step 9 did not make.

It is recorded here rather than quietly absorbed, and the history is left alone —
rewriting a published tag to tidy a provenance error would be a worse trade than
stating it.

## Verification

Gazebo bringup, then `move_group`, then a plan-and-execute through the same
`/move_action` interface RViz's Plan and Execute buttons use.

```text
ros2 control list_controllers
  arm_controller           joint_trajectory_controller/JointTrajectoryController  active
  joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
```

The robot state is now complete — nine joints from two publishers:

```text
arm joints seen:    7  [panda_joint1 ... panda_joint7]
finger joints seen: 2  [panda_finger_joint1, panda_finger_joint2]
total:              9
```

```text
"You can start planning now" lines:        1
"Missing panda_finger_joint1" occurrences: 0
```

Planning and executing `home` → `ready`:

```text
move_action server found
goal accepted
error_code: 1  (1 == SUCCESS)
trajectory points: 101
final trajectory point: -0.005, -0.785, -0.001, -2.352, +0.000, +1.562, +0.780
```

Those seven values are the `ready` pose from the SRDF, reached to within
0.01 rad. And the trajectory did not merely plan — it ran:

```text
[moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]:
    Controller 'arm_controller' successfully finished
```

That line is the whole milestone: MoveIt planned, handed the trajectory to the
controller named in Step 9's configuration, and the controller built in Step 8
executed it on the robot spawned in Step 4.

Build and lint across four packages: 11 tests, 0 errors, 0 failures, 1 skipped.

## Lessons

**Peel one gate at a time.** Five failures, each hidden behind the last. Nothing
was wrong with the approach; this is what integrating a planner with a simulator
looks like, and treating each error as a discovery rather than a setback is the
only way through it.

**"The build succeeded" is not "the code is there."** The `ament_python`
entry-point stub will happily install for a module that does not exist.

**A generated safety artefact can have gaps.** The collision matrix came from
sampling, and sampling misses things. It is a starting point to be checked
against the poses the robot actually holds, not a finished answer.

**A geometric path is not a trajectory.** Planners produce shape; timing is a
separate stage with its own inputs, and it fails for its own reasons.

## Known limitations

- **RViz's interactive marker never rendered.** Adding the end effector to the
  SRDF cleared the "no end effectors" complaint but did not make the drag handle
  appear. Not chased, because scripted Cartesian goals do not need it — but it
  means goals currently have to be set in joint space or in code.
- **Planner selection does not persist.** RViz returns to Pilz on every restart,
  and Pilz fails on an empty planner id. Both this and the planning group have to
  be reset by hand each session, or saved into `moveit.rviz`.
- **The fingers are reported, not actuated.** `finger_state_publisher` exists to
  complete the robot state for the planner. Real grasping needs the fingers wired
  into `<ros2_control>`.
- **Bringup is now three terminals** — Gazebo, `move_group`, RViz — with an
  ordering requirement between them.
- **`bridge.launch.py` is still vestigial** and still included.
- **No perception.** `move_group` logs missing 3D sensor and octomap plugins at
  startup; there is no camera and no collision environment beyond the robot.

## Commit boundaries

1. `feat(moveit): run move_group against the Gazebo control stack`
2. `fix(bringup): complete the robot state with finger joint publication`
3. `fix(moveit): unblock planning with collision pairs and an end effector`
4. `docs: document the MoveIt-to-Gazebo execution milestone`

## Next engineering step

Replace the GUI with a script. `move_to_named(name)` and `move_to_pose(x, y, z)`
as callable primitives are what a language model will eventually drive — and a
Cartesian goal is the first thing in this project that needs inverse kinematics.
