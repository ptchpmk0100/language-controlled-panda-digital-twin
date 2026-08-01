# Step 11 — Motion Primitives You Can Call

## Objective

Turn planning into an API. Step 10 moved the arm from a GUI; a language model
cannot press Plan. This milestone builds two callable primitives:

- `move_to_named(name)` — a joint-space goal from an SRDF pose. No IK.
- `move_to_pose(x, y, z)` — a Cartesian goal for `panda_link8`. Solved with IK.

The second one is the point. Everything so far has commanded joint angles;
natural language will ask for positions in space, and that is a different problem.

```bash
ros2 run twin_moveit_scripts move_to named ready
ros2 run twin_moveit_scripts move_to pose 0.28 -0.2 0.5
ros2 run twin_moveit_scripts move_to demo
```

A command-line interface was chosen deliberately: whatever drives these later can
invoke them the same way a person does.

## moveit_py is not a client

`moveit_py` embeds a MoveIt context **in this process**. It does not connect to a
running `move_group` — it *is* one. So this script replaces the standalone
`move_group` rather than talking to it, and running both means two planning
contexts issuing trajectories to the same controllers.

The Gazebo bringup still has to be running, because execution needs
`arm_controller` to exist.

## The four files a hand-built `ament_python` package needs

`ros2 pkg create` writes these; a package assembled by hand is the one that
forgets one.

| File | Why |
|---|---|
| `package.xml` | Declares the build type and dependencies |
| `setup.py` | Entry points and data files |
| **`setup.cfg`** | **Where console scripts get installed** |
| `resource/<pkg>` | The ament index marker |

`setup.cfg` is the one that bites:

```ini
[install]
install_scripts=$base/lib/twin_moveit_scripts
```

Without it, setuptools installs the console script into `bin/`, where neither
`ros2 run` nor a launch file looks. The build succeeds, the package appears
installed, and `ros2 pkg executables` returns nothing.

The check is a directory listing, not a build result:

```text
install/twin_moveit_scripts/lib/twin_moveit_scripts/     -> move_to
install/twin_moveit_scripts/bin/                         -> does not exist
```

## Three things `moveit_py` needs spelled out

Each of these fails in a way that does not name its cause.

### The pipeline configuration has two shapes at once

`MoveItConfigsBuilder` produces `planning_pipelines` as a flat list of names.
`moveit_cpp` reads the *names* from `planning_pipelines.pipeline_names`, but reads
each pipeline's plugin configuration from that pipeline's own **top-level**
namespace. Both layouts have to be present simultaneously:

```python
config['planning_pipelines'] = {'pipeline_names': pipeline_names}   # nested
# the top-level 'ompl' block stays exactly where the builder put it
```

Getting this wrong yields "Planning plugin name is empty or not defined in
namespace 'ompl'", which sounds like a missing plugin rather than a misplaced key.

Restricting the builder to OMPL alone is also deliberate: it otherwise discovers
CHOMP, STOMP and Pilz, and `moveit_cpp` aborts if any configured pipeline fails to
load.

### The plan request needs a pipeline named on it

```python
params = PlanRequestParameters(robot, 'ompl')
params.planning_pipeline = 'ompl'
params.planner_id = 'RRTConnect'
```

Without them, `plan()` fails with "No planning pipeline available for name ''".
There is no implicit default.

### `use_sim_time` has exactly one delivery route that works

This is the same clock seam that has appeared in every recent step, and here it
has two wrong answers and one right one:

| Route | Result |
|---|---|
| `config_dict=` | Crashes: `qos_overrides./clock.subscription.durability could not be set` |
| Launch-file node parameters | Never reaches the embedded node; joint states then look perpetually stale and execution refuses to validate |
| `create_params_file_from_dict(config, '/**')` + `launch_params_filepaths=` | Works |

The `/**` wildcard root applies the parameter to every node in the process,
including the embedded one that cannot be addressed by name.

### And one race

The trajectory-execution action client needs a moment to connect to
`arm_controller` after the context is built. Executing immediately fails with
"Action client not connected", and `ros2 run` starts fast enough to hit it where
`ros2 launch` did not. A short sleep after construction covers it.

## Verification

Gazebo bringup running, no standalone `move_group`.

**Joint-space goal.** `move_to named ready`:

```text
=== move_to_named('ready') ===
Planning succeeded, executing...
Controller 'arm_controller' successfully finished
Completed trajectory execution with status SUCCEEDED
move_to_named('ready') -> ok
```

Measured against the SRDF pose:

| Joint | `ready` | Measured | Error |
|---|---|---|---|
| `panda_joint1` | +0.000 | −0.0000 | 0.0000 |
| `panda_joint2` | −0.785 | −0.7850 | 0.0000 |
| `panda_joint3` | +0.000 | +0.0000 | 0.0000 |
| `panda_joint4` | −2.356 | −2.3560 | 0.0000 |
| `panda_joint5` | +0.000 | −0.0000 | 0.0000 |
| `panda_joint6` | +1.571 | +1.5710 | 0.0000 |
| `panda_joint7` | +0.785 | +0.7850 | 0.0000 |

**Cartesian goal.** `move_to pose 0.28 -0.2 0.5`, with the resulting
`panda_link0` → `panda_link8` transform read back from TF rather than trusting the
request:

```text
target   x=+0.280  y=-0.200  z=+0.500
measured x=+0.2797 y=-0.2008 z=+0.4997
error    0.0009 m
```

Under a millimetre, through IK the script never solved itself — that is KDL
working through the configuration built in Step 9.

Package installation, which is the other thing this step can get wrong:

```text
install/.../lib/twin_moveit_scripts/  ->  move_to
install/.../bin/                      ->  does not exist
ros2 pkg executables twin_moveit_scripts  ->  twin_moveit_scripts move_to
```

Build and lint across five packages: 11 tests, 0 errors, 0 failures, 1 skipped.

## Lessons

**Read the error's cause, not its noun.** "Planning plugin name is empty"
described a missing plugin; the actual fault was a dictionary key one level too
deep. Three of this step's failures named the wrong subsystem.

**A configuration API can have a shape its consumer does not accept.** The
builder and `moveit_cpp` disagree about where pipeline configuration lives, and
both are internal to the same project.

**The same seam keeps reappearing at every new node.** `use_sim_time` has now
caused a failure at the controller manager, at `robot_state_publisher`, at
`move_group`, and here. Each time it presented differently. It is worth checking
first, not last, whenever a new node joins the stack.

**Startup order is a real interface.** The action client connects asynchronously,
so "constructed" does not mean "able to execute."

## Known limitations

- **The primitives return a bare boolean.** Whether the arm ended up where it was
  asked is not reported, and a caller cannot distinguish a planning failure from
  an execution failure.
- **No orientation control.** `move_to_pose` sets identity orientation, so the
  planner picks any hand angle that satisfies the position. Adequate for reaching
  a point, not for grasping.
- **No reachability check.** An unreachable target is a planning failure with no
  useful explanation. Nothing validates the request against the workspace first.
- **Exactly one MoveIt context may exist.** Running this alongside
  `move_group_gazebo.launch.py`, or twice at once, puts two planners on the same
  controllers.
- **A segmentation fault on teardown**, after `Done.` — work already complete, a
  shutdown-ordering problem in `moveit_py`. Cosmetic, but it means the process
  exit code is not trustworthy.
- **The planner falls back on a lookup it cannot find**, logging "Cannot find
  planning configuration for group 'panda_arm' using planner 'RRTConnect'". OMPL
  uses the same planner anyway; an `ompl_planning.yaml` would silence it.
- **Still no perception, and the gripper is still inert.**

## Commit boundaries

1. `feat(scripts): add moveit_py motion primitives for the Panda`
2. `docs: document the scripted motion milestone`

## Next engineering step

Give the primitives a language front end: a local model emitting constrained JSON
tool calls that map onto `move_to_named` and `move_to_pose`, with argument
validation before anything reaches the robot.
