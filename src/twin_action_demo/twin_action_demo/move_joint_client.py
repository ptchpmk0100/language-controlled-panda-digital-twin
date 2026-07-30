#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from twin_interfaces.action import MoveJoint


class MoveJointClient(Node):
    """Send one joint-angle goal and report feedback and result."""

    def __init__(self) -> None:
        super().__init__('move_joint_client')

        self.declare_parameter('target_angle', 1.57)

        self._action_client = ActionClient(
            self,
            MoveJoint,
            '/move_joint',
        )

        self._goal_handle = None
        self.done = False

    def send_goal(self) -> None:
        """Wait for the server and asynchronously send one goal."""
        target_angle = float(
            self.get_parameter('target_angle').value
        )

        self.get_logger().info(
            f'Preparing target angle: {target_angle:.2f} rad'
        )

        while rclpy.ok():
            server_available = self._action_client.wait_for_server(
                timeout_sec=1.0
            )

            if server_available:
                break

            self.get_logger().info(
                'Waiting for /move_joint action server...'
            )

        if not rclpy.ok():
            self.done = True
            return

        goal_message = MoveJoint.Goal()
        goal_message.target_angle = target_angle

        self.get_logger().info(
            f'Sending goal: {target_angle:.2f} rad'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_message,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future) -> None:
        """Handle server acceptance or rejection."""
        try:
            self._goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to send goal: {error}'
            )
            self.done = True
            return

        if not self._goal_handle.accepted:
            self.get_logger().warning('Goal rejected by server.')
            self.done = True
            return

        self.get_logger().info('Goal accepted by server.')

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message) -> None:
        """Receive progress information while motion executes."""
        feedback = feedback_message.feedback

        self.get_logger().info(
            'Feedback | '
            f'current={feedback.current_angle:.2f} rad, '
            f'remaining={feedback.remaining:.2f} rad'
        )

    def result_callback(self, future) -> None:
        """Process the final action result."""
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self.get_logger().error(
                f'Failed to receive result: {error}'
            )
            self.done = True
            return

        if result.success:
            self.get_logger().info(
                'Motion succeeded | '
                f'final_angle={result.final_angle:.2f} rad'
            )
        else:
            self.get_logger().warning(
                'Motion did not complete | '
                f'final_angle={result.final_angle:.2f} rad'
            )

        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[MoveJointClient] = None

    try:
        node = MoveJointClient()
        node.send_goal()

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
