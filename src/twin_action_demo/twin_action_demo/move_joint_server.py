import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64

from twin_interfaces.action import MoveJoint

# Command topic consumed by the JointPositionController plugin declared in
# twin_description/urdf/one_joint_arm.urdf. Carries a target angle in radians.
COMMAND_TOPIC = '/model/one_joint_arm/joint/joint1/cmd_pos'

# Measured state published by the JointStatePublisher plugin and bridged into
# ROS 2. The world name is part of the topic because Gazebo scopes model topics
# by world.
JOINT_STATE_TOPIC = '/world/empty/model/one_joint_arm/joint_state'

JOINT_NAME = 'joint1'

# Radians. A PID controller settles with a small steady-state error that varies
# with pose, because the gravity torque at the target differs. This tolerance
# accepts that error rather than waiting for an exact match that never arrives.
ARRIVAL_TOLERANCE = 0.05

# Seconds. Bounds the motion so a stalled controller aborts the goal instead of
# blocking the executor thread forever.
ARRIVAL_TIMEOUT = 10.0

# Seconds. Bounds the wait for the first measurement, so a missing bridge or a
# paused simulator is reported rather than hanging the goal.
FIRST_MEASUREMENT_TIMEOUT = 10.0

# Seconds between feedback publications while waiting for arrival.
POLL_PERIOD = 0.1


class MoveJointServer(Node):

    def __init__(self):
        super().__init__('move_joint_server')

        # Latest measured angle in radians; None until the first joint state
        # arrives from the simulator.
        self.latest_position = None

        # The execute callback blocks while polling, so it must not share a
        # thread with the subscription that supplies the value it polls. A
        # reentrant group on a MultiThreadedExecutor lets both run at once;
        # under the default single-threaded executor the subscription is
        # starved and the poll loop waits on a value that can never update.
        callback_group = ReentrantCallbackGroup()

        self.command_publisher = self.create_publisher(
            Float64,
            COMMAND_TOPIC,
            10,
        )

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
            callback_group=callback_group,
        )

        self._server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=callback_group,
        )
        self.get_logger().info('MoveJoint server ready.')

    def joint_state_callback(self, msg):
        """Store the measured angle of the controlled joint."""
        if JOINT_NAME in msg.name:
            self.latest_position = msg.position[msg.name.index(JOINT_NAME)]

    def goal_callback(self, goal_request):
        target = goal_request.target_angle
        if abs(target) > 3.14:
            self.get_logger().warn(
                f'Rejecting target {target:.2f} (out of bounds).'
            )
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting target {target:.2f}.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — accepting.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_angle
        feedback = MoveJoint.Feedback()

        if not self._wait_for_first_measurement():
            self.get_logger().error(
                f'No joint state on {JOINT_STATE_TOPIC}; '
                'is the simulator running and the bridge up?'
            )
            goal_handle.abort()
            return self._result(success=False, final_angle=0.0)

        # Publish the setpoint once. Holding position is the controller's job
        # from here; this node only observes the outcome.
        command = Float64()
        command.data = target
        self.command_publisher.publish(command)
        self.get_logger().info(
            f'Commanded {target:.2f}, waiting for arrival...'
        )

        deadline = time.monotonic() + ARRIVAL_TIMEOUT

        while abs(self.latest_position - target) > ARRIVAL_TOLERANCE:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Motion canceled.')
                return self._result(
                    success=False, final_angle=self.latest_position
                )

            if time.monotonic() > deadline:
                self.get_logger().warn(
                    f'Timed out at {self.latest_position:.2f} rad before '
                    f'reaching {target:.2f} rad.'
                )
                goal_handle.abort()
                return self._result(
                    success=False, final_angle=self.latest_position
                )

            # Feedback reports the measured angle, not a predicted one.
            feedback.current_angle = self.latest_position
            feedback.remaining = abs(target - self.latest_position)
            goal_handle.publish_feedback(feedback)
            time.sleep(POLL_PERIOD)

        goal_handle.succeed()
        self.get_logger().info(f'Reached {self.latest_position:.2f}.')
        return self._result(success=True, final_angle=self.latest_position)

    def _wait_for_first_measurement(self):
        """Block until a measured angle is available, or the wait expires."""
        deadline = time.monotonic() + FIRST_MEASUREMENT_TIMEOUT

        while self.latest_position is None:
            if time.monotonic() > deadline:
                return False
            self.get_logger().info('Waiting for first joint state...')
            time.sleep(POLL_PERIOD)

        return True

    @staticmethod
    def _result(success, final_angle):
        result = MoveJoint.Result()
        result.success = success
        result.final_angle = final_angle
        return result


def main():
    rclpy.init()
    node = MoveJointServer()
    # Pairs with the ReentrantCallbackGroup above: the polling execute callback
    # and the joint-state subscription need separate threads.
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
