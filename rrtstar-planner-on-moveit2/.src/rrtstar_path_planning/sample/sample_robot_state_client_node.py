
from rrtstar_path_planning.robot_state_client_node import (
    RobotStateClientNode
)

import rclpy

def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger("rrtstar_path_planning")
    logger.info("ROS 2 initialized")

    robot_state_node = RobotStateClientNode()
    logger.info("RobotStateClientNode initialized")
    joint_limits = robot_state_node.get_joint_limits()
    logger.info(f"Joint limits received: {joint_limits}")

    logger.info("MotionPlanner initialized")

    robot_state_node.destroy_node()
    logger.info("Node destroyed")

    rclpy.shutdown()
    logger.info("ROS 2 shutdown complete")


if __name__ == "__main__":
    main()
