# Step 9 — The Planning Layer

## Objective

Describe the robot a second time.

The URDF and `panda_controllers.yaml` describe how joints are actuated and read.
A planner needs entirely different facts about the same machine: which joints form
an arm, how to solve inverse kinematics for it, which link pairs can never
collide, and how to hand a finished plan to something that will execute it.

That is the MoveIt configuration package, and it is this milestone's deliverable.
Running `move_group` against Gazebo is the next step, not this one.

## Two layers, one seam

| Layer | Files | Answers |
|---|---|---|
| Control | `panda.urdf`, `panda_controllers.yaml` | How is a joint commanded and read? |
| Planning | `panda.srdf`, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml` | What is an arm, where can it go, how is a plan executed? |

They meet at exactly one string — the name of the controller.

```yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
```

MoveIt executes a plan by sending a `FollowJointTrajectory` goal to
`/arm_controller/follow_joint_trajectory`. Step 8 built a controller named
`arm_controller` that advertises exactly that action. The name was chosen then for
this moment.

**The Setup Assistant does not know that.** Its "auto add" names the MoveIt
controller after the *planning group*, producing `panda_arm_controller` — a
plausible-looking name for a controller that does not exist. Renaming it to
`arm_controller` is the single most consequential edit in the whole package,
because a mismatch here does not fail loudly: MoveIt plans successfully, sends the
goal to a nonexistent action, and execution simply never happens.

## Generated, not hand-written — and why

The self-collision matrix is the reason. It records which link pairs never
meaningfully collide, so the planner can skip checking them, and it is built by
sampling thousands of random poses and observing what actually touches. Thirty-five
pairs were disabled here, each with a reason:

| Reason | Meaning |
|---|---|
| `Adjacent` | The pair shares a joint |
| `Never` | Never collided across the sampled poses |
| `Default` | Overlapping in the zero pose |
| `Always` | Overlapping in every pose |

Hand-authoring thirty-five pairs is tedious, and errors are asymmetric: a pair
wrongly disabled tells the planner it is safe to drive the arm through itself.
Sampling density was raised to ~50000 to buy confidence that "Never" really means
never. This is the one artefact where a GUI genuinely beats writing it by hand.

## What was deliberately skipped

Three of the Setup Assistant's steps were left alone on purpose, and each is a
decision rather than an omission.

**No virtual joint.** The tutorial for the stock Panda adds one, because that
description is rooted at `panda_link0` and needs to be tied to the world. This
robot already has a `world` link and a fixed joint to `panda_link0`, added back in
Step 4. Adding a virtual joint on top would describe the same constraint twice.
The tutorial was checked against the actual robot rather than copied.

**No end effector, no `hand` group.** Grasping is not on the menu yet, and the
planning group is the seven arm joints. Both are reversible later through the
assistant's "Edit Existing" mode.

**The "ROS 2 Controllers" tab was left empty.** That tab generates a
`ros2_controllers.yaml` for a standalone fake-hardware demo, complete with its own
controller manager. This project already has a real, verified controller stack
driving `GazeboSimSystem`. Generating a second one would produce two competing
definitions of the same joints. Only the *MoveIt Controllers* tab — the seam
described above — was configured.

The same logic applies to the generated `demo.launch.py`: it stands up fake
hardware, its own controllers, `move_group`, and RViz all at once. It ships with
the package and is not what this project will use.

## Correction made while committing this milestone

The Setup Assistant records the author's name and email into `package.xml` and
`.setup_assistant`, and it had captured a personal address. Replaced with the
no-reply address the repository's other packages already use. Generated files are
still files, and "the tool wrote it" does not exempt them from review.

## Verification

The deliverable is a configuration, so the check is that MoveIt can load it and
that it describes the right robot. Built with `colcon build`, then loaded through
the same `MoveItConfigsBuilder` the launch files use:

```text
### parameter sets the builder produced
    robot_description                present
    robot_description_semantic       present
    robot_description_kinematics     present
    joint_limits                     present
    trajectory_execution             present
    planning_pipelines               present
```

**The URDF MoveIt loads is the one Gazebo runs**, which matters because two
different Panda descriptions would put the planner and the simulator in different
coordinate frames:

```text
    ros2_control contract    found
    gazebo system plugin     found
    world anchor             found
```

The planning layer itself:

```text
    groups                    ['panda_arm']
    chain                     panda_link0 -> panda_link8
    named states              ready, home
    end effectors             none (deliberate)
    disabled collision pairs  35
    IK solver                 kdl_kinematics_plugin/KDLKinematicsPlugin
```

A kinematic chain implies its joints, so `panda_arm` expands to
`panda_joint1..7` without the joint list being written out.

And the seam, which is the part that has to match Step 8 exactly:

```text
    controller_names          ['arm_controller']
    arm_controller.type       FollowJointTrajectory
    arm_controller.action_ns  follow_joint_trajectory
    arm_controller.joints     7 joints
```

Install copy confirmed on disk: all six config files and eight launch files.
Build and lint across four packages: 11 tests, 0 errors, 0 failures, 1 skipped.

## Lessons

**The same robot needs different descriptions for different jobs**, and keeping
them consistent is a real engineering task rather than duplication to be
eliminated. They are allowed to overlap; they are not allowed to disagree.

**Generated code still has to be read.** Every file was inspected, and the one
that mattered most — the controller name — was wrong by default in a way that
would have failed silently at execution time.

**A default that looks reasonable is more dangerous than one that looks wrong.**
`panda_arm_controller` is exactly what someone would name that controller. Nothing
about it invites suspicion.

**Check the tutorial against your robot.** The virtual joint is required for the
stock Panda and redundant here, and the difference is visible only by looking at
what this URDF actually roots itself on.

## Known limitations

- **Nothing has been planned yet.** `move_group` has not been run, no motion has
  been planned or executed, and the seam to `arm_controller` is verified by
  configuration rather than by a trajectory arriving at the controller.
- **`demo.launch.py` ships but must not be used** against this project: it stands
  up fake hardware and a competing controller manager.
- **KDL is the IK solver**, chosen as the default. It can fail to converge near
  joint limits; TRAC-IK is a one-line change in `kinematics.yaml` if that turns out
  to matter.
- **No end effector is configured**, so the gripper cannot be planned for, and
  grasping is out of reach until the SRDF gains a `hand` group.
- **Re-running the Setup Assistant may overwrite hand edits** to generated files —
  including the controller rename. Each file is now either assistant-owned or
  hand-owned, and that distinction is not recorded anywhere the tool can see.
- **Joint limits are scaled to 10%** of maximum velocity and acceleration by
  default, so planned motions will be deliberately slow until that is raised.

## Commit boundaries

1. `feat(moveit): add the MoveIt configuration package`
2. `docs: document the MoveIt planning-layer milestone`

## Next engineering step

Run `move_group` against the existing Gazebo stack — not the generated
`demo.launch.py` — with `use_sim_time` set, then plan and execute a motion through
`arm_controller` and watch the arm move in the simulator.
