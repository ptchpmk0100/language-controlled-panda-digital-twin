# Step 14 — Report Actual, Not Asked

## Objective

Let a caller react to a motion. The primitives returned a bare boolean, which
cannot say *why* a motion failed or *where* the arm ended up — so nothing built on
top of them could do anything but log.

This is a small change that unlocks a category: retrying, sequencing, verifying,
correcting all need an outcome to inspect.

## A typed result

```python
@dataclass
class MoveResult:
    ok: bool
    stage: str                     # 'executed' | 'plan_failed' | 'exec_error'
    final_pose: Optional[Tuple[float, float, float]] = None
    error: Optional[str] = None
```

A boolean cannot carry why or where. A `(bool, dict)` tuple can, but its keys are
a convention that rots the first time someone adds a field. A dataclass gives one
shape both files agree on, and new fields — planning time, distance to goal — go
in without touching a single call site.

`stage` is the field that earns its place. "Failed" is not actionable; *"the
planner never produced a trajectory"* and *"execution broke partway"* call for
different responses. The first suggests trying a different target, the second
suggests stopping and looking at the robot.

## The pose has to be measured, not echoed

`final_pose` is read from the robot's live current state through forward
kinematics, after the motion settles:

```python
state = robot.get_planning_component(ARM_GROUP).get_start_state()
transform = state.get_global_link_transform(END_EFFECTOR_LINK)
```

Echoing the requested goal would have been simpler, and would have made the field
worthless. Asked and actual diverge in exactly the cases anyone would want to
react to — a clamped target, drift, a partially executed motion. A result that
reports the request is a lie in precisely the situations where the truth matters.

**A diagnostic must not be able to invent a failure.** If the pose read fails it
returns `None` and logs a warning; it never turns a successful motion into a
reported failure. The secondary concern is contained inside itself.

## The test oracle

`ready` and `home` are **named joint poses**. No Cartesian coordinate is ever
supplied to the primitive, anywhere in the call.

So if the result reports coordinates, they can only have come from forward
kinematics on the live state. There is no other source for them. That makes the
named-pose path a self-verifying test of the read-back — which matters, because
`get_global_link_transform` returning a 4×4 and the translation living in its
last column were the one thing that could not be checked without a robot.

## Verification

Three instructions, through the prompt:

```text
[repl] LLM -> move_to_named {'name': 'ready'}
    move_to_named('ready') -> ok @ (0.307, -0.000, 0.590)
    [repl] DONE: ok @ (0.307, -0.000, 0.590)

[repl] LLM -> move_to_named {'name': 'home'}
    move_to_named('home')  -> ok @ (0.088, -0.000, 0.926)
    [repl] DONE: ok @ (0.088, -0.000, 0.926)

[repl] LLM -> move_to_pose {'x': 0.3, 'y': -0.2, 'z': 5.0}
    clamped z: 5.0 -> 0.9 (bounds 0.15..0.9)
    move_to_pose -> ok @ (0.299, -0.200, 0.900)
    [repl] DONE: ok @ (0.299, -0.200, 0.900)
```

The first two are the oracle firing: real coordinates for goals that carried
none. The third is the other half — the reported pose is the *clamped* target,
0.900, not the 5.0 that was asked for, which is what "report actual" means in
practice.

Independently, from TF rather than from the result:

```text
panda_link8 at x=+0.2994 y=-0.2002 z=+0.9003
```

Agreement to within a millimetre between the robot's own report and an outside
measurement.

Build and lint across five packages: 14 tests, 0 errors, 0 failures, 2 skipped.

## Lessons

**Return a type, not a flag,** as soon as anything downstream must react.
Retrofitting a richer result later means touching every call site; starting with
one costs a dataclass.

**A result that echoes the request is worse than no result**, because it looks
like evidence. The whole value of a reported pose is that it can disagree with
the goal.

**Contain secondary failures.** A diagnostic that can fail must fail into "no
information", never into "the operation failed."

**Look for the self-verifying case.** Reporting xyz for a joint-space goal is a
test that needs no assertions — no xyz was available to fake it with.

## Known limitations

- **Nothing consumes the result yet.** `handle_instruction` logs it and returns
  it; no retry, no sequencing, no verification against the goal exists.
- **`ok` means "planned and executed without raising"**, not "arrived". Nothing
  compares `final_pose` against the target, so a motion that stopped short still
  reports success.
- **Orientation is not reported.** `final_pose` is position only, which is
  sufficient for reaching a point and not for grasping.
- **The settle before sampling is a fixed sleep**, not a check that motion has
  stopped.
- **Planner parameters are silently not taking effect.** Every plan logs
  `Cannot find planning configuration for group 'panda_arm' using planner
  'RRTConnect'` and `Parameter 'ompl.plan_request_params...' not found`. Planning
  succeeds on OMPL defaults, so the velocity and acceleration scaling set in the
  code may not be applied at all. Harmless today; it means "success" is currently
  hiding a misconfiguration.

## Commit boundaries

1. `feat(scripts): report a typed result carrying the live end-effector pose`
2. `docs: document the richer motion results milestone`

## Next engineering step

Give the arm something to do with its hand. The gripper is still reported but not
actuated, and grasping is the first task that needs a result richer than
"reached" — closing on nothing and closing on an object look identical unless the
controller reports the difference.
