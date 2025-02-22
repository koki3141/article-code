import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__("robot_state_publisher_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.joint_state_publisher = self.create_publisher(
            JointState, "/lbr/joint_states", 10
        )

    def publish_joint_states(self, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"]
        if len(positions) < 7:
            positions.extend([0.0] * (7 - len(positions)))

        message.position = positions
        message.velocity = [0.0] * len(message.name)
        message.effort = [0.0] * len(message.name)
        self.joint_state_publisher.publish(message)
        self.get_logger().info("Joint states published.")
        time.sleep(0.5)


