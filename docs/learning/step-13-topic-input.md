# Step 13 — A Second Way In

## Objective

Accept instructions from a ROS topic as well as the prompt, so the input can come
from something that is not a keyboard — a speech front end, a planner, another
node. The prompt stays, because it is the fastest way to test.

```bash
ros2 topic pub --once /voice_command std_msgs/String "data: 'go home'"
```

Two inputs, one robot. That is the whole design problem.

## One dispatch, two callers

The translate-clamp-dispatch sequence lives in a single `handle_instruction`,
called by both paths. Duplicating it into the subscriber callback would have been
smaller today and would have drifted the moment either copy changed — a clamp
fixed in one place and not the other is exactly the kind of divergence that only
shows up when it matters.

The `source` argument tags every log line `[repl]` or `[topic]`, so which path
drove a motion is visible rather than inferred.

## Threading, and the seam it creates

The prompt blocks the main thread on `input()`, so the subscriber needs its own
node spun in a background thread. That immediately creates a hazard the
single-threaded program did not have: two sources can now issue motions at once,
into one MoveIt context.

```python
with lock:
    ...dispatch...
```

The lock wraps the **motion**, not the translation. Two instructions can be
translated concurrently — that is just two HTTP requests — but only one can drive
the arm, and a second arriving mid-move waits its turn. Serializing matches how
the prompt already behaved; dropping the newer command was the alternative and is
a one-line change if it ever becomes preferable.

## The failure that ate the session

The symptom: `ros2 topic pub` hung on *"Waiting for at least 1 matching
subscription(s)"*, and `ros2 topic info /voice_command --verbose` reported
**Subscription count: 0** — while the prompt kept working perfectly.

The cause was not the topic name, or the QoS, or the message type. The first
version referenced `rclpy.executors.SingleThreadedExecutor` while importing only
`rclpy` and `rclpy.node`. `rclpy.executors` is not guaranteed to be imported as a
side effect, so the thread target raised `AttributeError` on its first line — and
**a daemon thread that raises simply vanishes.** No traceback, no exit code, no
message. The main thread carried on serving the prompt, entirely unaware that the
subscriber had never been created.

Two changes came out of it. The class is imported directly, and every thread
target is wrapped:

```python
def spin():
    try:
        executor.spin()
    except Exception as error:
        logger.error(f'spin thread died: {error}')
```

The subscriber's construction is wrapped too, so a future failure degrades to
"the prompt still works" with a logged reason instead of a silently missing
feature.

**"Subscription count: 0 with a publisher present" points at the subscriber
process**, not at the topic. The publisher is behaving correctly by waiting; the
question is why the other side never registered.

## Verification

Bringup running, `llm_move` started, instructions published from a third
terminal.

The subscriber exists — this is the number that was zero during the bug:

```text
ros2 topic info /voice_command --verbose
    Subscription count: 1
```

And its startup lines are now present rather than absent:

```text
Subscribed to /voice_command (std_msgs/String).
Topic path started (spin thread live).
```

Two published instructions, dispatched through the shared path:

```text
[topic] LLM -> move_to_named {'name': 'ready'}
    Controller 'arm_controller' successfully finished

[topic] LLM -> move_to_pose {'x': 0.3, 'y': -0.2, 'z': 5.0}
    clamped z: 5.0 -> 0.9 (bounds 0.15..0.9)
    Controller 'arm_controller' successfully finished
```

The clamp fires on this path too, which is the point of sharing the dispatch
rather than reimplementing it. Measured from TF afterwards:

```text
panda_link8 at x=+0.3001 y=-0.1997 z=+0.9004
```

Build and lint across five packages: 14 tests, 0 errors, 0 failures, 2 skipped.

## A note on this repository's history

The session factored `handle_instruction` out of the prompt loop as part of this
step. In this repository it was already a separate function in Step 12, so this
commit adds the topic path, the lock and the source tagging, but contains no
extraction diff.

## Lessons

**Background threads fail silently.** A `threading.Thread` whose target raises
disappears without a trace, and the program keeps looking healthy. Every thread
target needs a `try`/`except` that logs, because the alternative is diagnosing a
missing feature from its downstream symptom.

**Diagnose from the side that is missing.** The publisher was waiting correctly.
Everything about the publisher was fine. The investigation only moved once it
turned to why the subscriber had never registered.

**Adding a caller is a concurrency change.** The second input path introduced no
new logic and a new failure mode. The lock is not defensive programming; it is
the reason two inputs can share one arm at all.

## Known limitations

- **Serialize, never drop.** A burst of commands queues up and executes in order,
  so a stale instruction can move the arm long after it was sent. There is no
  timeout and no way to cancel a queued command.
- **No feedback on either path.** A topic publisher gets no acknowledgement, no
  result, and no error. It cannot know whether the arm moved.
- **The topic is unauthenticated and unvalidated at the ROS layer** — anything
  that can publish `std_msgs/String` can drive the robot, subject only to the
  clamp.
- **Both paths share one process**, so an LLM-side crash still takes MoveIt down.
- **`std_msgs/String` carries no structure** — no sender, no timestamp, no
  request id — so nothing can be correlated with a result later.

## Commit boundaries

1. `feat(llm): accept instructions from /voice_command alongside the prompt`
2. `docs: document the topic input milestone`

## Next engineering step

Give the primitives something worth reporting. They currently return a bare
boolean, so neither path can tell a caller whether the arm reached the target, or
why it did not — which is the prerequisite for anything that reacts to an outcome.
