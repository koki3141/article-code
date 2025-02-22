
from rrtstar_path_planning.robot_state_client_node import (
    RobotStateClientNode
)
from rrtstar_path_planning.robot_state_publisher_node import (
    RobotStatePublisherNode
)
import rclpy
import numpy as np

from rrt_algorithms.rrt.rrt_star import RRTStar
from rrt_algorithms.search_space.search_space import SearchSpace


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger("lbr_iiwa7_rrtstar_path_planing")
    logger.info("ROS 2 initialized")

    robot_state_node = RobotStateClientNode()
    logger.info("RobotStateClientNode initialized")
    joint_limits = robot_state_node.get_joint_limits()
    logger.info(f"Joint limits received: {joint_limits}")

    logger.info("MotionPlanner initialized")

    if joint_limits['success']:
            # Extract joint limits data
            min_positions = joint_limits['data'].min_position
            max_positions = joint_limits['data'].max_position
            # Create X_dimensions array with joint limits
            X_dimensions = np.array([(min_pos, max_pos) for min_pos, max_pos in zip(min_positions, max_positions)])

            x_init = (
                1.8500490071139892,
                0.12217304763960307,
                -0.4014257279586958,
                -1.1693705988362009,
                0.5410520681182421,
                0.3141592653589793,
                0.0)  # starting location
            x_goal = (
                2.530727415391778,
                -1.3089969389957472,
                -1.7453292519943295,
                -1.3089969389957472,
                -1.2217304763960306,
                1.4835298641951802,
                0.0)  # goal location

            q = 0.06  # length of tree edges
            r = 1  # length of smallest edge to check for intersection with obstacles
            max_samples = 2**15  # max number of samples to take before timing out
            rewire_count = 32  # optional, number of nearby branches to rewire
            prc = 0.0  # probability of checking for a connection to goal

            # create Search Space
            X = SearchSpace(X_dimensions)

            # create rrt_search
            rrt = RRTStar(X, q, x_init, x_goal, max_samples, r, prc, rewire_count)
            path = rrt.rrt_star()
            robot_state_publisher_node =  RobotStatePublisherNode()
            for point in path:
                robot_state_publisher_node.publish_joint_states(point)
    else:
        logger.error("Failed to get joint limits, using default values")


    robot_state_node.destroy_node()
    logger.info("Node destroyed")

    rclpy.shutdown()
    logger.info("ROS 2 shutdown complete")


if __name__ == "__main__":
    main()
