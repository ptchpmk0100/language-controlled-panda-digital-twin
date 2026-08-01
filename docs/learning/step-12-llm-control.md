# Step 12 — Natural Language Drives the Arm

## Objective

Close the loop the project is named after. A typed English instruction becomes a
tool call, the tool call becomes a motion, and the motion happens on the robot.

This is the milestone where an unreliable component gets wired to a physical
system, so most of the work is about what sits between them.

```text
"move to x 0.3 y -0.2 z 5.0"
        │
        ▼  local LLM, decoding constrained to a JSON schema
{"action": "move_to_pose", "args": {"x": 0.3, "y": -0.2, "z": 5.0}}
        │
        ▼  clamp against the reachable box
{"x": 0.3, "y": -0.2, "z": 0.9}     ← z was out of reach
        │
        ▼  move_to_pose primitive from Step 11
arm moves
```

## Why the model is small

`qwen2.5:3b-instruct`, running locally through Ollama, entirely on a 4 GB GPU
that also has a simulator on it. A 7B model does not fit and spills to CPU.

This is not a compromise. The job is to turn one sentence into one of two tool
calls with three numeric arguments — classification with argument extraction, not
reasoning. A larger model would cost latency and memory to do the same thing.

## Constrained decoding, and why the schema shape matters

Ollama's `format` field accepts a JSON schema and constrains decoding to it. The
model cannot emit prose, a missing key, or an unexpected one; the shape is
enforced *while tokens are generated*, not validated afterwards. Parsing the
reply cannot fail.

That guarantee is about **shape**, and the first schema proved how narrow a
guarantee that is. With a permissive argument object:

```json
"args": {"type": "object"}
```

the model read *"go to the ready position"* as a `move_to_pose` carrying an
invented `pose_name` key. The schema permitted it, so constrained decoding
faithfully produced it. The output was perfectly valid and completely wrong.

The fix binds each action to its exact argument shape:

```json
{"oneOf": [
  {"properties": {"action": {"const": "move_to_named"},
                  "args": {"required": ["name"], "additionalProperties": false}}},
  {"properties": {"action": {"const": "move_to_pose"},
                  "args": {"required": ["x","y","z"], "additionalProperties": false}}}
]}
```

`const` pins the action, `required` demands the right keys, and
`additionalProperties: false` forbids invented ones. Choosing `move_to_pose` now
*requires* producing three numbers, which makes the wrong choice structurally
unavailable rather than merely discouraged.

A sharper system prompt naming the exact argument keys was also needed. Neither
change alone was sufficient — the schema removes impossible outputs, the prompt
improves the choice among possible ones.

## The clamp is the safety seam

A schema constrains shape, never values. Nothing upstream stops the model
proposing a point two metres away, and "the model is usually right" is not a
safety property.

```python
BOUNDS = {'x': (0.15, 0.65), 'y': (-0.5, 0.5), 'z': (0.15, 0.9)}
```

Every Cartesian target is snapped into that box, and any change is logged rather
than applied silently. The bounds are deliberately smaller than the arm's true
workspace: the cost of refusing a valid target is one retry, the cost of
accepting an invalid one is a failed plan or a collision.

Named poses need no clamping — the schema's `enum` already restricts them to
`ready` and `home`, which is a case where shape *is* the whole constraint.

The principle: the model proposes, the code disposes.

## One context, imported not spawned

Three ways to connect a language model to the primitives were available:

| Approach | Cost |
|---|---|
| Subprocess per command | Rebuilds the MoveIt context every call, pays the settle each time, string-only I/O |
| **Import the primitives** | **One shared context; a crash takes both down** |
| ROS service or action | Fully decoupled processes; needs an interface definition and two nodes |

The middle one was taken. Step 11's primitives already accept live handles rather
than reaching for globals, so importing them was nearly free. That same property
is what would make the service version mostly interface plumbing rather than a
rewrite, if process isolation becomes worth having.

## Verification

**Translation**, at temperature 0 — the same sentence must always produce the
same call:

| Instruction | Tool call |
|---|---|
| `go to the ready position` | `move_to_named {'name': 'ready'}` |
| `go home` | `move_to_named {'name': 'home'}` |
| `reset the arm` | `move_to_named {'name': 'home'}` |
| `move to x 0.3 y -0.2 z 0.5` | `move_to_pose {'x': 0.3, 'y': -0.2, 'z': 0.5}` |
| `put the gripper at 0.4, 0.1, 0.6` | `move_to_pose {'x': 0.4, 'y': 0.1, 'z': 0.6}` |

The last two matter most: neither phrasing matches the prompt's wording, and
`reset the arm` names no pose at all yet resolves to `home`.

**Clamping**, in isolation:

| Input | Output | Logged |
|---|---|---|
| `{0.3, -0.2, 0.5}` | unchanged | — |
| `{0.3, -0.2, 5.0}` | `z -> 0.9` | `clamped z: 5.0 -> 0.9` |
| `{-2.0, 9.9, -1.0}` | `{0.15, 0.5, 0.15}` | one warning per axis |

**End to end**, with the simulator running and three instructions typed at the
prompt:

```text
LLM -> move_to_named {'name': 'ready'}
    Controller 'arm_controller' successfully finished
    move_to_named('ready') -> ok

LLM -> move_to_pose {'x': 0.3, 'y': -0.2, 'z': 0.5}
    Controller 'arm_controller' successfully finished
    move_to_pose -> ok

LLM -> move_to_pose {'x': 0.3, 'y': -0.2, 'z': 5.0}
    clamped z: 5.0 -> 0.9 (bounds 0.15..0.9)
    Controller 'arm_controller' successfully finished
    move_to_pose -> ok
```

And where the arm actually finished, read from TF:

```text
panda_link8 at x=+0.2994 y=-0.2003 z=+0.9006
```

That last line is the milestone in one measurement. A sentence asked for a point
five metres in the air; the arm is at 0.9006 m, because the clamp stood between
the request and the robot.

Build and lint across five packages: 14 tests, 0 errors, 0 failures, 2 skipped.

## A note on this repository's history

The session records a refactor of `move_to.py` into setup / primitives / teardown
seams as part of this step. That structure was already present in this
repository's Step 11 implementation, written that way from the start rather than
arrived at by refactoring. There is no refactor commit here because there was
nothing to refactor — Step 12 adds only the language layer.

## Lessons

**A guarantee about shape is not a guarantee about meaning.** Constrained
decoding made the output always parseable and, with a loose schema, reliably
wrong. Encoding the *relationships* between fields — this action implies these
arguments — is what made it correct.

**Make wrong answers unrepresentable rather than unlikely.** Prompting a model
not to invent a key is a request. A schema that forbids the key is a property.

**Validate at the boundary you control.** The LLM is the untrusted component, so
the check belongs immediately after it and before the robot — not inside the
model, and not deep in the motion primitive.

**Small models expose sloppy interfaces.** The 3B model's misclassification was a
real defect in the schema. A larger model would likely have papered over it, and
the interface would still have been wrong.

## Known limitations

- **Typed input only.** One instruction at a time, from a prompt, in one process.
- **Clamping hides intent.** An unreachable request is silently moved to the
  nearest reachable point and executed. The user is told through a log line, and
  nothing asks whether that substitution was wanted.
- **No feedback to the user.** The primitives return a bare boolean and the loop
  only logs; the operator cannot see whether the arm reached the target.
- **No orientation, no multi-step plans.** One position, one call. "Pick up the
  block" has nowhere to go.
- **The LLM shares the process with MoveIt.** A crash in either takes both down.
- **Two named poses**, `ready` and `home`, hard-coded in the schema and the prompt.
- **No conversation memory.** Each instruction is translated in isolation, so
  "now move it left" cannot work.
- **Ollama must be running**, and the model name is a constant in the source.

## Commit boundaries

1. `feat(llm): drive the arm from natural language via constrained tool calls`
2. `docs: document the natural-language control milestone`

## Next engineering step

Accept instructions from a ROS topic as well as the prompt, so the input can come
from something other than a keyboard — with a single shared dispatch path, and a
lock, because two sources can now issue motions at once.
