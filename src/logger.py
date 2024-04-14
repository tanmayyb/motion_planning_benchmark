#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateLogger(Node):

    def __init__(self):
        super().__init__('joint_state_logger')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        self.get_logger().info("Joint Positions:")
        # for name, position in zip(msg.name, msg.position):
        #     self.get_logger().info(f"{name}: {position:.2f}")
        self.get_logger().info(f"{msg.name} {msg.position}")

def main(args=None):
    rclpy.init(args=args)
    joint_state_logger = JointStateLogger()
    rclpy.spin(joint_state_logger)
    joint_state_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()