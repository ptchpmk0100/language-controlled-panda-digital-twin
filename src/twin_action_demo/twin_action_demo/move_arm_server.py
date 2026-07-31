import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64

from twin_interfaces.action import MoveArm

# Goal order. Index i of target_angles is JOINTS[i].
JOINTS = [f'panda_joint{i}' for i in range(1, 8)]

# Measured state for the whole model arrives on one topic; commands go out on
# one topic per joint.
JOINT_STATE_TOPIC = '/world/empty/model/panda/joint_state'

# Radians. Matches the one-joint server: a PID controller settles with a small
# pose-dependent offset, and waiting for an exact match would never finish.
ARRIVAL_TOLERANCE = 0.05

# Seconds. Longer than the one-joint arm's budget because a seven-joint goal is
# gated by whichever joint has the furthest to travel.
ARRIVAL_TIMEOUT = 15.0

# Seconds. Bounds the wait for the first full measurement so a missing bridge
# or a paused simulator is reported rather than hanging the goal.
FIRST_MEASUREMENT_TIMEOUT = 10.0

# Seconds between feedback publications while waiting for arrival.
POLL_PERIOD = 0.1

# Radians. Matches the limit the description declares on every arm joint.
JOINT_LIMIT = 3.14


class MoveArmServer(Node):

    def __init__(self):
        super().__init__('move_arm_server')

        # Measured angles keyed by joint name; empty until the first message.
        self.latest = {}

        # Same reasoning as the one-joint server: the execute callback blocks
        # while polling, so it cannot share a thread with the subscription that
        # supplies the values it polls.
        callback_group = ReentrantCallbackGroup()

        self.command_publishers = {
            joint: self.create_publisher(Float64, f'/{joint}/cmd_pos', 10)
            for joint in JOINTS
        }

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
            callback_group=callback_group,
        )

        self._server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=callback_group,
        )
        self.get_logger().info('MoveArm server ready.')

    def joint_state_callback(self, msg):
        """Store the measured angle of every arm joint, looked up by name."""
        # The message also carries the world joint and the fingers, and its
        # ordering is not guaranteed, so a fixed index would silently read the
        # wrong joint.
        for joint in JOINTS:
            if joint in msg.name:
                self.latest[joint] = msg.position[msg.name.index(joint)]

    def goal_callback(self, goal_request):
        targets = goal_request.target_angles

        if len(targets) != len(JOINTS):
            self.get_logger().warn(
                f'Rejecting goal: got {len(targets)} angles, '
                f'need {len(JOINTS)}.'
            )
            return GoalResponse.REJECT

        if any(abs(target) > JOINT_LIMIT for target in targets):
            self.get_logger().warn(
                'Rejecting goal: an angle is outside '
                f'+/-{JOINT_LIMIT} rad.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting goal: {[round(t, 2) for t in targets]}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — accepting.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        targets = list(goal_handle.request.target_angles)
        feedback = MoveArm.Feedback()

        if not self._wait_for_first_measurement():
            missing = [j for j in JOINTS if j not in self.latest]
            self.get_logger().error(
                f'No measurement for {missing} on {JOINT_STATE_TOPIC}; '
                'is the simulator running and the bridge up?'
            )
            goal_handle.abort()
            return self._result(success=False)

        for joint, target in zip(JOINTS, targets):
            command = Float64()
            command.data = target
            self.command_publishers[joint].publish(command)
        self.get_logger().info(
            f'Commanded all {len(JOINTS)} joints, waiting for arrival...'
        )

        deadline = time.monotonic() + ARRIVAL_TIMEOUT

        def errors():
            return [
                abs(self.latest[joint] - target)
                for joint, target in zip(JOINTS, targets)
            ]

        # The slowest joint gates success: the goal is reached only once every
        # joint is inside the tolerance band.
        while max(errors()) > ARRIVAL_TOLERANCE:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Motion canceled.')
                return self._result(success=False)

            if time.monotonic() > deadline:
                worst = JOINTS[errors().index(max(errors()))]
                self.get_logger().warn(
                    f'Timed out before all joints reached target; '
                    f'{worst} is furthest out.'
                )
                goal_handle.abort()
                return self._result(success=False)

            feedback.current_angles = [self.latest[j] for j in JOINTS]
            feedback.remaining = errors()
            goal_handle.publish_feedback(feedback)
            time.sleep(POLL_PERIOD)

        goal_handle.succeed()
        self.get_logger().info(f'All {len(JOINTS)} joints reached target.')
        return self._result(success=True)

    def _wait_for_first_measurement(self):
        """Block until every arm joint has been measured, or the wait expires."""
        deadline = time.monotonic() + FIRST_MEASUREMENT_TIMEOUT

        while not all(joint in self.latest for joint in JOINTS):
            if time.monotonic() > deadline:
                return False
            self.get_logger().info('Waiting for first full joint state...')
            time.sleep(POLL_PERIOD)

        return True

    def _result(self, success):
        result = MoveArm.Result()
        result.success = success
        result.final_angles = [
            self.latest.get(joint, 0.0) for joint in JOINTS
        ]
        return result


def main():
    rclpy.init()
    node = MoveArmServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
