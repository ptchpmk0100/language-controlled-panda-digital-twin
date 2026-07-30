import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from twin_interfaces.action import MoveJoint


class MoveJointServer(Node):

    def __init__(self):
        super().__init__('move_joint_server')
        self.current_angle = 0.0
        self._server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )
        self.get_logger().info('MoveJoint server ready.')

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
        step = 0.05 if target >= self.current_angle else -0.05

        while abs(self.current_angle - target) > abs(step):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Motion canceled.')
                result = MoveJoint.Result()
                result.success = False
                result.final_angle = self.current_angle
                return result

            self.current_angle += step
            feedback.current_angle = self.current_angle
            feedback.remaining = abs(target - self.current_angle)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        self.current_angle = target
        goal_handle.succeed()
        result = MoveJoint.Result()
        result.success = True
        result.final_angle = self.current_angle
        self.get_logger().info(f'Reached {target:.2f}.')
        return result


def main():
    rclpy.init()
    node = MoveJointServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
