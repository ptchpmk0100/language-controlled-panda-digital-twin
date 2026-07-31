# Step 3 — Command the Simulated Joint and Feed Real State Back

## Objective

Close the loop. Step 1 built an action whose execution callback advanced a Python
variable by `0.05` each tick. Step 2 put a physically simulated joint on the other
side of a bridge but had no way to drive it. This milestone deletes the fake
increment: an action goal now becomes a real command to a real controller, and the
feedback the client prints is a measurement taken from the physics engine.

It also retires the four-terminal ritual behind a launch file.

## Outcome

```text
move_joint_client 0.8                    (positional argument, radians)
        │  MoveJoint goal
        ▼
move_joint_server  execute_callback
        │  std_msgs/Float64 on /model/one_joint_arm/joint/joint1/cmd_pos
        ▼
ros_gz_bridge   ROS → gz   (the ']' direction)
        │  gz.msgs.Double
        ▼
JointPositionController plugin  ──►  PID drives joint1 under real physics
        │
        ▼
JointStatePublisher plugin  ──►  gz topic
        │
        ▼
ros_gz_bridge   gz → ROS   (the '[' direction)
        │  sensor_msgs/JointState
        ▼
move_joint_server  publishes the MEASURED angle as action feedback
```

Every arrow is real. Nothing in the path is simulated in Python any more.

## What changed

### `twin_description/urdf/one_joint_arm.urdf` — a second `<gazebo>` block

Step 2's description carried one plugin, `JointStatePublisher`, which reports state
outward. Commanding needs its own plugin, and it needs its own block:

```xml
<gazebo>
  <plugin
      filename="gz-sim-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
    <joint_name>joint1</joint_name>
    <topic>/model/one_joint_arm/joint/joint1/cmd_pos</topic>
    <p_gain>10.0</p_gain>
    <i_gain>5.0</i_gain>
    <d_gain>1.0</d_gain>
    <i_max>10.0</i_max>
    <i_min>-10.0</i_min>
  </plugin>
</gazebo>
```

**The identity of a plugin is the `filename`/`name` pair** — which shared object, and
which C++ class inside it. Everything below that pair is configuration handed to that
class. The first attempt pasted the controller's `joint_name` and gains *inside* the
existing `JointStatePublisher` block. It built, it spawned, and it did nothing: the
state publisher ignored the tags it did not recognise, and no controller was ever
instantiated. Two plugins means two blocks.

The command topic carries a `gz.msgs.Double` — one scalar in radians, not a
`JointState` — which the bridge maps to `std_msgs/Float64`.

### `twin_action_demo/move_joint_server.py` — measurement replaces arithmetic

The execute callback now publishes the setpoint once and then watches:

1. wait for the first `JointState` before commanding anything;
2. publish the target on `cmd_pos` — holding position is the controller's job from
   there, not this node's;
3. poll the measured angle until it is within `ARRIVAL_TOLERANCE`, publishing the
   *measured* value as feedback each cycle;
4. succeed, or abort on timeout.

**The concurrency bug this exposes is the real lesson.** A polling execute callback
under the default single-threaded executor cannot work. The callback occupies the
only thread, so the subscription that supplies `latest_position` never runs, so the
value being polled can never change, so the loop spins until it times out. The fix is
a `ReentrantCallbackGroup` shared by the action and the subscription, spun by a
`MultiThreadedExecutor`:

```python
callback_group = ReentrantCallbackGroup()
...
executor = MultiThreadedExecutor()
rclpy.spin(node, executor=executor)
```

This also settles a limitation recorded against Step 1. That milestone registered a
cancel callback but could not service it while the execute callback was blocking; the
README declined to claim working cancellation. With the executor and callback group in
place, the cancel path can now actually be reached mid-motion.

### `twin_action_demo/move_joint_client.py` — the client was ignoring its argument

`ros2 run twin_action_demo move_joint_client 1.0` moved the arm to 1.57. So did
`move_joint_client 0.5`. So did every other invocation.

**ROS 2 nodes do not read `sys.argv` the way ordinary programs do.** The client read
its target from a *parameter* that defaulted to 1.57, and the idiomatic way to set that
is `--ros-args -p target_angle:=1.0`. The positional argument was never parsed by
anything, so it was silently discarded — and because 1.57 is a plausible-looking
result, every test appeared to work.

The fix accepts both forms. `remove_ros_args()` strips ROS's own arguments so they do
not collide with argparse, the positional argument becomes the parameter's default, and
an explicit `--ros-args -p` still overrides it:

```python
parsed = parser.parse_args(remove_ros_args(args)[1:])
...
self.declare_parameter('target_angle', target_angle)
```

### `twin_action_demo/launch/arm_bringup.launch.py` — new

Spawn, bridge, and action server in one `ros2 launch`. Four terminals become two.

Gazebo itself is deliberately **not** launched here. The simulator needs the vendored
Gazebo variables exported into its terminal, and Step 2 decided to keep those out of
ROS-only terminals; folding `gz sim` into this file would drag that environment along
with it.

**Substitutions are the concept worth carrying forward.** The first version hardcoded
an absolute path into the author's own home directory. The replacement is:

```python
urdf_path = PathJoinSubstitution([
    FindPackageShare('twin_description'), 'urdf', 'one_joint_arm.urdf',
])
```

A substitution is *a promise to compute a value later* — resolved when the launch
system starts the node, not while `generate_launch_description()` runs. That is
precisely why `os.path.join` cannot be used on it. MoveIt's launch files are built
almost entirely from substitutions, so the mental model pays for itself later.

Launch files only exist in the install space if `setup.py` puts them there:

```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
```

## Correction made while committing this milestone

The session recorded gains of `p=10.0, i=0.1, d=1.0` and treated the resulting
steady-state error as an accepted characteristic — "varies with pose, because the
gravity torque at the target differs." Re-running the milestone headless before
committing showed that this is not merely cosmetic: **a 0.80 rad goal settled at
0.905 rad and the action aborted**, because the 0.105 rad error is larger than the
0.05 rad arrival tolerance.

The number is not mysterious. A proportional controller produces force in exchange for
error, so it must hold an error to hold a load:

```text
gravity torque at 0.8 rad  =  m·g·r·sin(θ)  =  0.5 × 9.81 × 0.3 × sin(0.8)  ≈  1.05 N·m
steady-state error         =  τ / p_gain    =  1.05 / 10                     ≈  0.105 rad
```

Eliminating a constant offset is exactly what the integral term is for, but raising
`i_gain` alone only improved it to ~0.045 rad. The reason is a second default: the
plugin clamps its integral contribution to ±1, which is below the 1.47 N·m this link
demands at its worst pose, so the term saturates before it can finish the job. Setting
`i_max`/`i_min` to the joint's own effort limit removes the clamp as the binding
constraint.

| Configuration | Command 1.57 rad | Settled | Error |
|---|---|---|---|
| `i=0.1`, default clamp | 1.57 | ~0.905 at a 0.80 goal | 0.105 rad, goal aborted |
| `i=5.0`, default clamp | 1.57 | 1.6137 | 0.044 rad |
| `i=5.0`, `i_max=10.0` | 1.57 | 1.5701 | 0.00008 rad |

The committed description uses the last row. This is a repository-time correction, not
something the session concluded.

## Verification

Headless, on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim 8, in a shell sourcing only
`/opt/ros/jazzy/setup.bash` and this repository's install space.

```text
colcon build
  Summary: 3 packages finished [2.45s]

colcon test-result --verbose
  Summary: 11 tests, 0 errors, 0 failures, 1 skipped
```

`gz sim -s -r empty.sdf`, then `ros2 launch twin_action_demo arm_bringup.launch.py`:

```text
### gz-side topics
/model/one_joint_arm/joint/joint1/cmd_pos
/world/empty/model/one_joint_arm/joint_state

### the same topics, bridged into ROS 2
/model/one_joint_arm/joint/joint1/cmd_pos
/world/empty/model/one_joint_arm/joint_state

### action server
/move_joint
```

Three goals, each followed by a settled measurement read back from the bridged topic:

| Goal | Result | Settled position | Error |
|---|---|---|---|
| `0.8` | `Motion succeeded \| final_angle=0.84 rad` | 0.8072 | 0.007 rad |
| `1.57` | `Motion succeeded \| final_angle=1.55 rad` | 1.5655 | 0.005 rad |
| `-1.0` | `Motion succeeded \| final_angle=-1.05 rad` | -1.0176 | 0.018 rad |
| `5.0` | `Goal rejected by server.` | — | rejected before motion |

The reported `final_angle` differs slightly from the settled value because the action
returns the instant the measurement enters the tolerance band, while the integral term
is still converging. The bounds check from Step 1 still rejects out-of-range goals
before anything is published.

## Lessons

**A plugin that loads is not a plugin that runs.** The nested-configuration mistake
produced no error at any stage — build, spawn, and launch were all clean. The only
symptom was absence. Confirm the effect, not the absence of complaints.

**"It worked" is not evidence when the expected answer is the default.** Every early
client test moved the arm to 1.57 rad, which looked correct because 1.57 was the
default. A test that cannot distinguish success from a stuck default is not a test.

**Blocking work needs an executor that can afford it.** Single-threaded execution is
the default and is fine for callbacks that return promptly. Anything that waits on
another callback's output needs the reentrant-group/multi-threaded pairing.

**Defaults have physics consequences.** The integral clamp was never mentioned, never
logged, and silently determined whether the milestone worked.

## Industrial relevance

- Command and measurement travel on separate, typed interfaces rather than shared
  program state — the boundary a real controller sits behind.
- Feedback reports what was measured, not what was intended. The distinction is what
  makes the value useful for diagnosis.
- The goal is bounded in time and aborts on failure rather than blocking indefinitely.
- Bringup is a single reproducible command with no machine-specific paths, so the
  milestone can be re-run by someone who did not build it.

## Known limitations

- **One joint.** The pipeline is proven, not the manipulator.
- **The arrival tolerance is fixed at 0.05 rad** and lives in the source rather than
  in a parameter. It is a reasonable band for this arm, not a general choice.
- **Gains are baked into the description**, so tuning requires an edit plus a
  `colcon build`. `ros2_control` externalises gains into YAML; that is a later step.
- **The result reports the first in-tolerance measurement**, not the fully settled
  angle.
- **Gazebo is still started by hand**, deliberately, to keep the vendored simulator
  environment out of ROS-only terminals.
- **Cancellation is now reachable but is not covered by an automated test.**

## Commit boundaries

1. `feat(description): drive the one-joint arm with a Gazebo PID controller`
2. `feat(actions): command real physics and report measured joint state`
3. `feat(bringup): launch spawn, bridge, and action server together`
4. `docs: document the closed-loop joint control milestone`

## Next engineering step

Graduate to the real robot. The MoveIt Panda description was rejected by Gazebo back in
Step 2 for carrying no `<inertial>` blocks; that is the obstacle to clear before any of
this pipeline can be pointed at a 7-DOF arm.
