import rclpy
from rclpy.node import Node

from robot_state_interface.srv import (
    JointLimit,
)


class RobotStateClientNode(Node):
    def __init__(self):
        super().__init__("robot_state_client_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.joint_limit_client = self.create_client(JointLimit, f"joint_limits")

    def get_joint_limits(self):
        if not self.joint_limit_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint limit service not available.")
            return {"success": False, "data": {}}

        request = JointLimit.Request()
        future = self.joint_limit_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().debug("Joint limits successfully retrieved.")
            return {"success": True, "data": response}
        else:
            self.get_logger().error("Failed to retrieve joint limits.")
            return {"success": False, "data": {}}

