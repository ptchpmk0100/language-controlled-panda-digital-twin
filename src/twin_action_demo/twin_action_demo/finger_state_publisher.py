"""
Publish state for the two gripper finger joints.

move_group builds its model from the full URDF, which has nine joints. The
joint state broadcaster reports only the seven listed in <ros2_control>, so the
fingers have no state source at all and the planning-scene monitor never sees a
complete robot. The symptom is a "missing panda_finger_joint1" message
repeating once a second, and no planning.

Ownership is what makes this safe: the broadcaster owns the seven arm joints,
this node owns the two fingers, and no joint is published by both. Subscribers
merge /joint_states by name, so two publishers of disjoint joints is fine.

This reports finger state; it does not command the fingers. They are not
actuated, and real grasping would need them wired into <ros2_control>.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 30.0

FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']

# Metres. Mid-range of the [0.0, 0.04] prismatic limit. panda_finger_joint2
# mimics joint1 with multiplier 1.0, so both carry the same value.
FINGER_POSITION = 0.02


class FingerStatePublisher(Node):

    def __init__(self):
        super().__init__('finger_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_state)

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = FINGER_JOINTS
        message.position = [FINGER_POSITION] * len(FINGER_JOINTS)
        message.velocity = [0.0] * len(FINGER_JOINTS)
        message.effort = [0.0] * len(FINGER_JOINTS)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FingerStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
