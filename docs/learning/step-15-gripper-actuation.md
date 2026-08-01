# Step 15 — Make the Gripper Real

## Objective

Step back from the obvious next brick.

Step 14 offered two follow-ons — retry on failure, or sequence multiple moves.
Measured against the actual goal of this project, language-driven pick and place,
neither was on the critical path. **There was no gripper to grasp with.** The
fingers were links, joints and a `<mimic>` present in every description and inert
in the simulator, faked into `/joint_states` by a stub so the planner would not
block.

So this milestone makes the fingers actually move. No Python was written; the
work is entirely in the description, the controller configuration, and the
bringup.

## One missing interface

The fingers had geometry, a joint, and a mimic relationship. What they did not
have was a `command_interface` — so nothing in `ros2_control` could drive them.
That single omission was the whole gap:

```xml
<joint name="panda_finger_joint1">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

Only `panda_finger_joint1` is listed. `panda_finger_joint2` carries a `<mimic>` of
it in the joint definition, and `gz_ros2_control` follows that on its own — a
second entry would make two owners of the same motion.

**Check how a joint is actuated before writing code against it.** `ros2 control
list_controllers`, `ros2 action list` and `/joint_states` answer "can this thing
move?" in seconds. Writing a gripper primitive first would have been the same
asked-versus-actual mistake one layer down: code that commands a joint nothing
can drive.

## Choosing a controller for what it *reports*

`GripperActionController`, not `JointTrajectoryController`. The arm's controller
would have worked for opening and closing, and would have been thrown away later,
because the distinguishing question about a grasp is not *"did the fingers reach
the width I asked for?"* but *"did they stop early because something is between
them?"*

```yaml
gripper_controller:
  ros__parameters:
    joint: panda_finger_joint1
    allow_stalling: true
    stall_velocity_threshold: 0.001
    stall_timeout: 0.5
```

`GripperCommand` carries a target width and an effort ceiling, and its result
reports whether the fingers **stalled**. Closing on an object and closing on air
are otherwise identical motions with identical commands.

`allow_stalling: true` is what makes that signal usable. Without it, a controller
that stalls on an object reports a *failure* — which is exactly backwards, since
stalling is how a gripper succeeds.

Two configuration details worth remembering: the parameter is `joint`, singular,
where the trajectory and group controllers take a `joints` list; and a missing
`joint` is the first thing to check if the controller will not activate.

## Retiring the stub

`finger_state_publisher` was removed from the bringup the moment the real
interface went live.

It existed only because the fingers had no command interface, so the broadcaster
never reported them and `move_group`'s scene monitor blocked on an incomplete
robot. Now the broadcaster reports the real joint — and the stub would be a
*second publisher of the same joint*, asserting a fixed 0.02 m against whatever
the finger is actually doing.

That is the same principle as Step 14's live pose read, applied to a placeholder:
a stand-in must not outlive the signal it stood in for, or it becomes a lie in a
new place.

## The lesson the session actually paid for

The gripper configuration was three small correct edits. Nearly all the effort
went into the same bug three times: **`src/` edits not reaching `install/`, which
is what the running simulator reads.**

An incremental `colcon build` does not reliably re-copy non-code assets — URDF,
YAML, launch files. The build reports success while the installed copy stays
stale, and the symptom appears somewhere else entirely: a joint missing from
`/joint_states`, a controller that silently will not spawn, a retired stub that
keeps respawning.

The rule, adopted here permanently:

```bash
rm -rf build/<pkg> install/<pkg>
colcon build --packages-select <pkg>
# and then, before launching anything, grep the INSTALLED copy
grep -n "<the thing you changed>" install/<pkg>/share/<pkg>/<path>
```

The gate is the second half. Verifying the source proves what you wrote;
verifying the install space proves what will run.

Three conditions had to hold *simultaneously* for the controller to come up — the
finger interface loaded, the controller type installed, and the launch file
spawning it. Each was individually true at different moments, which is precisely
why the failures were confusing.

## Verification

Clean rebuild, install-space gate, then a single launch:

```text
ros2 control list_controllers
  arm_controller           joint_trajectory_controller/JointTrajectoryController  active
  gripper_controller       position_controllers/GripperActionController           active
  joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active

ros2 action list
  /arm_controller/follow_joint_trajectory
  /gripper_controller/gripper_cmd

ros2 control list_hardware_interfaces
  panda_finger_joint1/position [available] [claimed]
```

The stub is gone and there is one honest source of finger state:

```text
finger_state_publisher processes: 0

joints in one message: 8
names: [panda_finger_joint1, panda_joint1 ... panda_joint7]
frame_id: 'base_link'
panda_finger_joint1 = 3.75691e-06        (the stub always said exactly 0.02)
distinct message shapes seen: [8]         (one shape == one publisher)
```

And the joint moves when commanded:

| Command | Controller reported | Live `/joint_states` | Stalled |
|---|---|---|---|
| open, 0.040 | 0.0300038 | 0.0399972 | False |
| close, 0.000 | 0.00999633 | −7.9e-17 | False |

`stalled=False` on the close is correct — there was nothing between the fingers.

The two columns disagreeing is worth noticing rather than smoothing over. The
controller reports its result the instant it declares the goal reached, while the
prismatic joint is still coasting; the live value settles afterwards. Anything
reading a final width should sample `/joint_states` after a settle, not trust the
result message — the same asked-versus-actual distinction as Step 14, arriving
from a new direction.

Build and lint across five packages: 14 tests, 0 errors, 0 failures, 2 skipped.

## Lessons

**Re-derive the next step from the goal, not from the last step.** The obvious
follow-ons were both reasonable and neither was on the path to pick and place.
The blocker was a capability that did not exist.

**Diagnose actuation before writing code against it.** "How is this joint driven?"
is a question with a live answer, and the answer here was "it isn't."

**Choose an interface for the information it returns.** A trajectory controller
could have opened and closed the fingers. Only the action controller can say
whether the grasp caught anything.

**Verify the install space, not the source.** `ros2 launch` runs the installed
copy, and an incremental build will happily leave it stale while reporting
success.

**Retire a placeholder the instant the real thing arrives**, or it becomes a
competing source of truth.

## Known limitations

- **No Python primitive yet.** The action is live and nothing in this project
  calls it; the gripper is driven only by hand from the command line.
- **The controller's reported position is not the settled position**, as the
  measurements above show.
- **Grasping is unproven.** `stalled` has only ever been observed as `False`,
  because nothing has been closed on an object. The signal the whole controller
  choice was made for has not been exercised.
- **`panda_finger_joint2` is followed by mimic in simulation only.** It has no
  interface of its own, and mimic support is a property of this physics engine
  and this build.
- **The retired stub's node definition remains in the launch file** as dead code,
  removed from the launch list but not from the file.
- **The gripper is outside the MoveIt planning group**, so plans cannot account
  for the fingers' state or open them as part of a motion.

## Commit boundaries

1. `feat(description): make the gripper an actuated joint`
2. `feat(bringup): spawn the gripper controller and retire the finger stub`
3. `docs: document the gripper actuation milestone`

## Next engineering step

Write the primitive that drives it: a `move_gripper(width)` that sends a
`GripperCommand`, waits for the result, and reports `stalled` — the grasp-success
signal that a `pick` sequence would consume.
