# Step 5 — Give the Panda Motors

## Objective

Step 4 left an anchored robot with no actuation: the base held, the arm folded.
This milestone applies the controller pattern proven on the one-joint arm to all
seven revolute joints, so the Panda holds its pose and accepts a commanded angle
per joint, and adds the state publisher that makes those joints measurable.

Scope stops there. The seven-way bridge and the action server that drives all
seven joints together belong to the next step; this one is verified with a
command-line bridge, which is enough to prove the plugins work.

## The multiplicity is the lesson

Two plugin kinds, two different multiplicities:

| Plugin | Multiplicity | Why |
|---|---|---|
| `JointPositionController` | **per joint** — 7 instances | Each runs a PID loop for one joint and listens on its own command topic |
| `JointStatePublisher` | **per model** — 1 instance | Walks every joint in the model and reports them all in one message |

The intuitive guess is that plugins come in matched pairs, one controller and one
publisher per joint. They do not. Seven controllers and one publisher — eight
`<gazebo>` blocks, not fourteen. Adding a publisher per joint would produce seven
copies of the same message.

The seven controller blocks are identical apart from two lines:

```xml
<gazebo>
  <plugin
      filename="gz-sim-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
    <joint_name>panda_joint4</joint_name>
    <topic>/panda_joint4/cmd_pos</topic>
    <p_gain>800</p_gain>
    <i_gain>20</i_gain>
    <d_gain>60</d_gain>
  </plugin>
</gazebo>
```

The gripper's prismatic finger joints are deliberately left unactuated, and
`panda_joint8` is fixed and has nothing to control.

## Gains

`p=800, i=20, d=60`, reached by tuning rather than derivation:

| Change | Behaviour |
|---|---|
| `p=800, i=20, d=15` | Rang badly — oscillated on arrival |
| `d=40` | Better, still visibly springy |
| `d=60` | Smooth travel |

The mental model that made the tuning tractable:

- **P is holding strength.** It buys force with error, so it always leaves a
  small offset under load.
- **D is braking during motion.** It governs how the joint arrives, not where it
  ends up. Ringing on arrival is a D problem.
- **I removes a constant offset**, but overdone it hunts around the setpoint —
  it keeps integrating after arrival and pushes past.

These gains are two orders of magnitude larger than the one-joint arm's `p=10`,
which is what carrying several kilograms of linkage instead of a 0.5 kg cylinder
costs.

## The transport wall

The controllers work before any ROS involvement at all:

```bash
gz topic -t /panda_joint2/cmd_pos -m gz.msgs.Double -p 'data: -0.5'
```

This is worth doing deliberately, because it separates two questions that look
identical from the ROS side: *is the controller working?* and *is the bridge
wired correctly?* Commanding straight over gz transport answers the first without
involving the second.

That separation exists because **gz plugins publish on gz transport, which ROS
cannot see**. `gz topic -l` lists all eight topics; `ros2 topic list` shows none
of them until a bridge runs. That gap is the entire reason `ros_gz_bridge`
exists.

## Verification

Headless, on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim 8. State is read
through a command-line `ros_gz_bridge`, which is the reading half of the loop:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/empty/model/panda/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
```

Eight topics, with the multiplicity visible in the listing:

```text
/panda_joint1/cmd_pos      /panda_joint5/cmd_pos
/panda_joint2/cmd_pos      /panda_joint6/cmd_pos
/panda_joint3/cmd_pos      /panda_joint7/cmd_pos
/panda_joint4/cmd_pos
/world/empty/model/panda/joint_state
```

**Holding.** Step 4's arm folded to its limits. After ten seconds of stepping
physics from the zero spawn pose:

| Joint | Held at | Joint | Held at |
|---|---|---|---|
| `panda_joint1` | +0.0013 | `panda_joint5` | −0.0569 |
| `panda_joint2` | +0.0042 | `panda_joint6` | −0.0017 |
| `panda_joint3` | −0.0012 | `panda_joint7` | −0.0721 |
| `panda_joint4` | +0.0028 | | |

**Tracking**, commanded one joint at a time over gz transport:

| Joint | Commanded | Measured | Error |
|---|---|---|---|
| `panda_joint2` | −0.5 | −0.5270 | 0.027 rad |
| `panda_joint4` | −1.5 | −1.5162 | 0.016 rad |
| `panda_joint6` | +1.0 | +0.9975 | 0.003 rad |

**Rest stability.** `panda_joint4` sampled three times at three-second intervals
after arriving: −1.5197, −1.5197, −1.5197. Identical to four decimal places.

## An experiment that was rejected

Step 3 found that this plugin clamps its integral contribution to ±1 by default,
and that raising `i_max`/`i_min` to the joint's own effort limit was what finally
removed the one-joint arm's steady-state droop. The same reasoning predicts an
improvement for `panda_joint5` and `panda_joint7`, which hold with visibly larger
offsets than the rest.

It was tried — per-joint clamps at the description's declared effort limits, 87
N·m for joints 1–4 and 12 N·m for joints 5–7 — and **not kept**:

| Configuration | `panda_joint5` | `panda_joint7` | `panda_joint4` at rest |
|---|---|---|---|
| Default clamp | −0.0569 | −0.0721 | −1.5197, −1.5197, −1.5197 |
| Clamp at effort limit | −0.0449 | −0.0604 | −1.5243, −1.5214, −1.5191 |

The offsets shrink slightly, and the joint stops being still — it creeps, which
is the integral winding on after arrival. That is precisely the hunting `i=20`
was already suspected of encouraging, and it trades a property that matters
(a joint that stays where it was put) for one that does not yet
(a hundredth of a radian of offset on two wrist joints).

Unlike Step 3, nothing here fails without the change; no goal aborts, because
nothing yet checks arrival against a tolerance. The recorded configuration is
kept, and the finding is recorded instead of the change.

## Lessons

**Multiplicity is part of an interface's design, not a detail.** Per-joint versus
per-model is a decision the plugin author made, and guessing it wrong produces
either a robot with no state or seven redundant publishers.

**Test the layer you are questioning.** Commanding over gz transport, with no
bridge running, is a one-line way to prove the controllers work — and to avoid
debugging a bridge that was never the problem.

**A tuning change that improves your metric can still be wrong.** The clamp
experiment improved the number being measured while degrading behaviour nobody
had thought to measure. Look at what got worse, not only at what got better.

**Scaling a proven pattern is not the same as learning a new one.** Going from
one joint to seven changed no concept — same plugin, same message type, same
topic convention. That is the return on having proved the pipeline on a robot
small enough to understand completely.

## Known limitations

- **`panda_joint5` and `panda_joint7` hold with ~0.05–0.07 rad of offset**,
  noticeably worse than the other five. Wrist joints have a 12 N·m effort limit
  against 87 N·m for the inner joints, and carry the welded hand mass.
- **Gains are baked into the description**, so every tuning change costs an edit
  and a `colcon build`. Externalising them is one of the reasons the next step
  moves to `ros2_control`.
- **Commands are seven independent scalars.** Nothing coordinates them, so there
  is no notion of a synchronised multi-joint motion, and nothing enforces joint
  limits before a command is published.
- **No ROS-side command path yet.** State is bridged for verification only; the
  seven-way bridge and the action server belong to the next step.
- **Self-collision is off**, which is the Gazebo default and is intentional.
  Packed links may interpenetrate; enabling it typically causes solver jitter
  during normal motion. Inter-link collision belongs at the planning layer, which
  arrives with MoveIt.
- **The gripper is still inert.**

## Commit boundaries

1. `feat(description): actuate all seven Panda arm joints`
2. `docs: document the seven-joint controller milestone`

## Next engineering step

Replace the command-line bridge with a seven-way bridge declared in a YAML config
and started from a launch file, then extend the action server to publish seven
targets and report seven-joint feedback — completing for the Panda what Step 3
completed for the one-joint arm.
