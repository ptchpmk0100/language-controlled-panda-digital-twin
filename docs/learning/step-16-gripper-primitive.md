# Step 16 — The Grasp-Success Signal

## Objective

Write the consumer of Step 15's gripper action: a `move_gripper(width)` that
commands the fingers, waits for the controller's verdict, and reports **whether
the grasp caught anything**.

Closing on an object and closing on air are the same command and the same
motion. The only thing that distinguishes them is what the controller reports,
which is why Step 15 chose a controller that reports it.

## A sibling result type, not a bigger one

```python
@dataclass
class GripResult:
    ok: bool
    stage: str                     # 'reached' | 'stalled' | 'rejected' | 'error'
    stalled: bool = False
    final_width: Optional[float] = None
    error: Optional[str] = None
```

Folding these fields into `MoveResult` was the obvious economy and would have
been wrong. `stalled` and `final_width` mean nothing for an arm move;
`final_pose` means nothing for a grip. One type carrying all of them is a shape
that lies about half its uses, and every reader would have to know which fields
apply to which call.

**`stalled` is the predicate; `final_width` is a diagnostic.** They stay separate
fields because they answer different questions — *is something held* versus
*where did the fingers stop* — and the verification below shows why conflating
them would mislead.

## An action needs its own node

The arm primitives lean on the node MoveItPy hides and already spins. An
`ActionClient` needs a node that *this* code can spin, and spinning MoveItPy's
would double-spin its current-state monitor and deadlock.

So `setup()` now builds a dedicated `twin_gripper` node, driven per call on its
own executor. `setup()` returns a five-tuple and `shutdown(twin)` destroys it.
That node is also the natural home for the non-MoveIt interfaces that grasping
and perception will need later.

One consequence worth knowing: a node belongs to one executor at a time, so
`read_finger_width` accepts the caller's executor rather than making its own.

## Getting an object between the fingers

The primitive was small. Placing a test object where the fingers actually are
took most of the effort, and the four traps are knowledge the next step needs.

**The TF root is `world`, not `panda_link0`.** `tf2_echo panda_link0
panda_leftfinger` warns that the frame does not exist and then prints numbers
anyway, from a stale resolution — a wrong answer that looks like an answer.

**`world` → `panda_link0` is identity.** Confirmed from `/tf_static`: the base
sits at the world origin with no offset or rotation. So fingertip coordinates in
the robot frame, world coordinates, and the `-x -y -z` arguments to `create` are
all the same numbers. That removes the frame-mismatch worry entirely.

**A live reading is valid only for the instant it was taken.** The fingertip
position wandered between reads with no commanded motion, because the arm had
settled differently after a relaunch or a failed plan. Each read was honest; the
mistake was placing an object using a read from a different arm pose. Move, read
once while settled, act — back to back, arm untouched.

**A yawed hand on an axis-aligned cube hits an edge.** The hand carries a
−0.785 rad yaw, so closing on a square box contacts a corner and cams it out
instead of stalling. The cube is spawned with a matching `-Y -0.785` so the
fingers meet flat faces.

## Verification

First, that the field being read actually exists:

```text
ros2 interface show control_msgs/action/GripperCommand
  bool stalled      # True iff the gripper is exerting max effort and not moving
```

The two outcomes the primitive exists to distinguish:

```text
closing on empty air    ->  ok [reached] width=-0.0000
closing on a 3 cm cube  ->  ok [stalled] width=0.0400 HELD(stalled)
```

One `reached`, one `stalled`, from the same command. The result is not
hard-coded, and `stalled` arrives as **success** rather than an abort — Step 15's
`allow_stalling` proven at the Python layer, not just in a YAML file.

**The stall width is not the object's thickness.** `width=0.0400` at stall looks
wrong until you notice the cube nearly fills the finger gap: the fingers met it
having barely left the open position. Stall width is *where the fingers were
stopped*, which depends on object size relative to the gap. This is exactly why
`stalled` and `final_width` are separate fields — reading grasp success off the
width would be wrong here.

## A defect this verification found

The session's testing covered opening into air and closing onto an object. It
did not cover **opening after a close**, and that case fails:

```text
start: live width 3.75691e-06
cycle 1  close cmd=0.00  before=+0.00000 after=-0.00000  stalled=False reached=True  effort=0
cycle 1  open  cmd=0.04  before=-0.00000 after=-0.00000  stalled=True  reached=False effort=10
cycle 2  close cmd=0.00  before=-0.00000 after=-0.00000  stalled=False reached=True  effort=10
cycle 2  open  cmd=0.04  before=-0.00000 after=-0.00000  stalled=True  reached=False effort=10
```

Once the gripper has been actively driven closed, it will not reopen. The finger
does not move at all, the controller reports `reached_goal=False` with effort
pinned at its ceiling, and it reports the failure as `stalled` — which the
primitive faithfully renders as a successful grasp of nothing.

Opening *does* work from the resting state it starts in, which is why Step 15's
check passed: that test opened first and closed second.

This matters beyond a cosmetic wart, because a pick sequence is
open → descend → close → lift, and a place sequence has to reopen to let go.
As it stands, the gripper is a one-shot.

A plausible cause, offered as a hypothesis rather than a finding: DART does not
support URDF mimic constraints — an error present since the Panda first spawned
— so `panda_finger_joint2` is an unconstrained passive joint rather than a
mirror of `panda_finger_joint1`. Confirming that, and deciding whether the fix is
a second command interface, a different controller, or a mimic-free description,
is its own piece of work and is not attempted here.

Build and lint across five packages: 14 tests, 0 errors, 0 failures, 2 skipped.

## Lessons

**A result type should not lie about its shape.** Two sibling dataclasses beat
one union whose fields are conditionally meaningful.

**Keep the predicate and the measurement separate.** Grasp success is `stalled`.
The width is a diagnostic that, in this very test, would have given the wrong
answer.

**Test both outcomes, and the reverse.** One `reached` and one `stalled` proves
the primitive distinguishes them. Neither proves the gripper can be *used*, which
takes a cycle — and the cycle is what failed.

**A live reading has an expiry.** The fingertip position is honest for the
instant it was sampled and misleading the moment the arm moves.

**Physics has to be running to stall.** A stall requires the fingers to load
against something; pausing the simulator freezes the arm too, so a scene can be
staged paused but never grasped paused.

## Known limitations

- **The gripper cannot reopen once closed** — see above. This blocks any pick or
  place sequence and is the most important thing on this page.
- **A failure to move is reported as a successful grasp.** `stalled` conflates
  "closed on an object" with "could not move", so the predicate is only
  trustworthy when the fingers were free to begin with.
- **`move_gripper` is not wired into the language layer.** The `twin` handle is
  threaded through `llm_move` so a gripper verb can reach it, but no verb was
  added; the gripper is reachable only from the command line.
- **Nothing composes the primitives.** `pick` does not exist; the approach,
  grasp and lift shown here were done by hand.
- **Object placement is manual and fragile**, depending on a fingertip read taken
  at the right instant and a hand-matched yaw.
- **The stall thresholds are untuned** — `stall_velocity_threshold` and
  `stall_timeout` were taken as written and are what make a stationary finger
  look like a grasp.
- **Carried, unchanged:** the planner parameters still silently fall back to OMPL
  defaults, and `move_to` still segfaults on teardown after its work completes.

## Commit boundaries

1. `feat(scripts): add a gripper primitive reporting the grasp signal`
2. `feat(description): add a graspable test object`
3. `docs: document the gripper primitive milestone`

## Next engineering step

Fix the reopen defect — without it there is no pick-and-place — and then compose
`pick(object_id)` from the pieces that now exist: approach, open, descend, close,
confirm the stall, lift.
