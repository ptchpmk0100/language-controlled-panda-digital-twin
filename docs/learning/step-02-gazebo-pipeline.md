# Step 2 — Get Real Joint State Out of Gazebo

## Objective

Prove the full simulation data path — description → physics → joint state → ROS 2 —
on the smallest robot that can exercise it. Step 1's action server moved a number in
Python. This milestone replaces that number's *source* with a physically simulated
joint, so a later step can feed measured state back as action feedback.

The deliverable is the pipeline, not the robot. A one-joint arm exercises exactly the
same spawn/bridge/echo path as a 7-DOF Panda.

## Outcome

| Package | Build type | Responsibility |
|---|---|---|
| `twin_description` | `ament_cmake` | Hold URDF models and install them into the share directory |

The completed path is:

```text
hand-written URDF (every link has <inertial>)
        │
        ▼
spawned into Gazebo Harmonic  ──►  joint1 swings under gravity (real dynamics)
        │
        ▼
JointStatePublisher system plugin  ──►  gz topic
        │
        ▼
ros_gz_bridge  (gz.msgs.Model → sensor_msgs/JointState)
        │
        ▼
/world/empty/model/one_joint_arm/joint_state readable in ROS 2
```

Nothing drives the joint at this stage. The arm falls and settles, which is the
correct behaviour for a robot with no motor.

## Why not the Panda yet

The intent was to jump straight to the real arm. That failed for an instructive reason,
recorded here so it is not re-litigated:

- `ros-jazzy-franka-description` does not exist in apt.
- `ros-jazzy-moveit-resources-panda-description` does exist and installs cleanly — but it
  is built for **motion planning and visualization**, so its links carry no `<inertial>`
  blocks. Gazebo is a physics engine and rejects massless links.
- The spawn reported `Entity creation successful.` and **nothing appeared**. The simulator
  terminal showed `link[panda_link0] has no <inertial> block` followed by
  `A model must have at least one link`.

There is no clean apt-installable, Gazebo-Harmonic-ready Panda for Jazzy; the community
options are clone-and-build against older ROS/Ignition releases.

**Decision:** build a minimal one-joint arm first, prove the pipeline, then scale to the
Panda. This mirrors Step 1's fake-joint-before-real-arm approach. The inertial requirement
discovered here is precisely what the Panda description must satisfy later.

## Session progression

1. Attempted to spawn the MoveIt Panda description directly; spawn "succeeded" but nothing
   appeared.
2. Read the simulator terminal and found the missing-`<inertial>` rejection.
3. Established that no apt-installable sim-ready Panda exists for Jazzy.
4. Pivoted to a hand-written one-joint arm.
5. Created `twin_description` as an `ament_cmake` package (it holds data, not nodes).
6. Wrote `one_joint_arm.urdf` with complete visual, collision, and inertial blocks.
7. Added an `install(DIRECTORY urdf ...)` rule so the description reaches the share directory.
8. Appended a `<gazebo>` pass-through block carrying the JointStatePublisher plugin.
9. Resolved the vendored-Gazebo environment so `gz` subcommands could load at all.
10. Spawned the arm and watched it swing down under gravity.
11. Confirmed the gz-side topic existed before touching the bridge.
12. Ran `ros_gz_bridge parameter_bridge` and echoed `sensor_msgs/JointState` in ROS 2.

## Files and what they teach

### `src/twin_description/urdf/one_joint_arm.urdf`

A base cube and a cylinder on a revolute joint. Each link carries three blocks, and the
distinction between them is the lesson:

| Block | Purpose |
|---|---|
| `<visual>` | what is rendered |
| `<collision>` | the shape physics uses for contact |
| `<inertial>` | mass and inertia tensor — **the block the Panda description omitted** |

The joint declares its rotation axis and limits:

```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
</joint>
```

The `±3.14` limit deliberately echoes the bounds check already enforced by Step 1's action
server, so the description and the software agree on the same constraint.

**Geometry-origin gotcha.** A `<cylinder>` is centred on its own origin, so a bare cylinder
would straddle the joint. Every block offsets `<origin xyz="0 0 0.3">` to stand the link up
from the joint. The offset is repeated in visual, collision, *and* inertial — omitting it
from one produces a robot that looks correct but behaves wrongly.

### The `<gazebo>` pass-through block

```xml
<gazebo>
  <plugin
      filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
  </plugin>
</gazebo>
```

URDF parsers ignore `<gazebo>`; Gazebo reads it. The plugin walks the model's joints each
simulation step and broadcasts them. Without it there is no joint state to bridge.

### `src/twin_description/CMakeLists.txt`

```cmake
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
```

ROS tools resolve package-relative paths through the **install space**, not the source
tree. Without this rule the URDF is invisible to `ros2 pkg prefix`-based lookups, and
editing the source URDF has no effect until `colcon build` copies it across.

## Environment lessons

### Gazebo naming

Ubuntu 24.04 + Jazzy pairs with **Gazebo Harmonic**, which is **Gazebo Sim 8**, driven by
the `gz` command. "Gazebo Classic" (`gazebo11`) is end-of-life. "Ignition" (`ign`) is the
same modern simulator under its interim name — stale documentation uses it freely. The ROS
bridge metapackage is `ros-jazzy-ros-gz`.

### Vendored Gazebo does not set up its own environment

Gazebo arrives through `-vendor` packages that install it *inside* the ROS tree
(`/opt/ros/jazzy/opt/gz_*_vendor/...`) rather than onto the system PATH. Verified in a
clean shell: sourcing ROS leaves `GZ_VERSION` empty. Four variables must be set manually:

```bash
export GZ_VERSION=harmonic
export PATH=$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin
export GZ_CONFIG_PATH=$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | tr '\n' ':')$LD_LIBRARY_PATH
```

The glob is `*_vendor`, **not** `gz_*_vendor` — `sdformat_vendor` supplies `libsdformat14.so`
and does not match the `gz_` prefix. That gap produced one full layer of the error stack.

These are sourced manually rather than from `.bashrc`, deliberately: it keeps roughly
eighteen vendor library directories off `LD_LIBRARY_PATH` in unrelated terminals, where a
vendored `.so` could shadow a system one. The working rule is that a terminal running `gz`
needs the Gazebo environment; a terminal running only ROS nodes does not.

### `gz` is a dispatcher, not a program

Each subcommand (`sim`, `topic`, `service`) is a separate binary in a different vendor
package, located through `GZ_CONFIG_PATH` manifests, whose libraries are then found through
`LD_LIBRARY_PATH`. This is why `gz` could appear installed while doing nothing useful. The
errors were peeled one layer at a time — `gui.so` → `tools-backward.so` → `sdformat14.so` →
success. **Errors that keep moving forward are evidence of progress**, not of thrashing.

## Debugging lessons

### Hollow success

`Entity creation successful.` was printed while the model was silently rejected. An INFO
line reports that a *request* was accepted, not that the outcome is correct. Trust the
viewport and the simulator's own terminal over a success message from the client.

### Confirm the producer before blaming the transport

Always check that the gz topic exists before debugging the bridge:

```bash
gz topic -l | grep joint
```

This separates "Gazebo is not publishing" from "the bridge is misconfigured". The two
failures look identical from the ROS side — an absent topic.

### The bridge argument is a type mapping, not a topic name

```text
/world/empty/model/one_joint_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model
        TOPIC                              @      ROS_TYPE           [   GZ_TYPE
```

`@` separates the fields; the bracket encodes direction — `[` is gz→ROS, `]` is ROS→gz, and
a lone `@` is bidirectional. This step needs only the read direction.

### A new terminal tab is not a clean shell

Tabs inherit the parent's environment, so a tab opened after sourcing the Gazebo variables
appears to have them automatically. Only a genuinely independent shell
(`env -i bash --norc`) tests whether configuration survives.

### Ignore the EGL warnings

Gazebo emits `libEGL warning: ... pci id ... 10de:1f91, driver (null)` and
`failed to create dri2 screen`. This is a hybrid-GPU laptop falling back from the discrete
NVIDIA card to integrated graphics. Rendering works. It is not worth pursuing.

## Verification

Reconstructed and re-run on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim 8.11.0, in a
shell sourcing only `/opt/ros/jazzy/setup.bash` and this repository's install space:

```text
colcon build
  Summary: 3 packages finished [2.24s]

colcon test-result --verbose
  Summary: 11 tests, 0 errors, 0 failures, 1 skipped

check_urdf one_joint_arm.urdf
  robot name is: one_joint_arm
  ---------- Successfully Parsed XML ---------------
  root Link: base_link has 1 child(ren)
      child(1):  arm_link
```

Headless simulation (`gz sim -s -r empty.sdf`), spawn, bridge, and echo:

```text
### spawning: .../install/twin_description/share/twin_description/urdf/one_joint_arm.urdf
[INFO] [ros_gz_sim]: Entity creation successful.

### gz topics matching joint:
/world/empty/model/one_joint_arm/joint_state

### sensor_msgs/JointState received:
header:
  stamp:
    sec: 17
    nanosec: 406000000
  frame_id: ''
name:
- joint1
position:
- 1.5760737886827674
velocity:
- 0.0016703074714352949
effort:
- 0.0
```

| Check | Observed |
|---|---|
| Description installs to share space | `one_joint_arm.urdf` present, 2142 bytes |
| URDF parses | `Successfully Parsed XML`, 2 links, 1 joint |
| Model spawns with physics | Entity created; joint moves without any command |
| Gazebo publishes joint state | gz topic present |
| Bridge converts to ROS 2 | `sensor_msgs/JointState` echoed |
| Joint settles under gravity | `position ≈ 1.576 rad` (≈ π/2), `velocity ≈ 0.0017` |

The arm comes to rest near π/2 — horizontal — which is where an unpowered link hanging off
a horizontal axis belongs. The near-zero velocity confirms it has settled rather than being
sampled mid-swing.

## Known limitations

- **Nothing commands the joint.** The arm only falls. There is no controller, so the
  position above is a consequence of gravity, not of a setpoint.
- **The bridge is one-way.** Only gz→ROS is configured; commanding requires the reverse
  direction, added in the next step.
- **The robot is a stand-in.** A one-joint arm proves the pipeline, not manipulation.
- **The pipeline is still manual.** Four terminals and a hand-typed bridge invocation. No
  launch file exists yet.

## Commit boundaries

1. `feat(description): add one-joint arm description for Gazebo physics`
2. `docs: document the Gazebo joint-state pipeline milestone`

## Next engineering step

Add a joint controller so the arm can be driven rather than dropped, and wire the Step 1
action server's execution callback to command that controller and consume the bridged
`/joint_state` as real feedback — replacing the fixed `+= 0.05` increment. That step also
introduces the reverse bridge direction and should retire the four-terminal ritual behind a
launch file.
