#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"

void displayJointLimits(
    const rclcpp::Logger& logger,
    const std::vector<moveit::core::JointModel*>& joint_models) {
  for (const auto* joint : joint_models) {
    if (joint->getVariableCount() > 0) {
      const std::string& joint_name = joint->getName();
      const std::vector<moveit::core::VariableBounds>& bounds =
          joint->getVariableBounds();

      if (!bounds.empty()) {
        const auto& bound = bounds[0];

        RCLCPP_INFO(logger, "Joint %s:", joint_name.c_str());
        RCLCPP_INFO(logger, "  Position limits: [%f, %f]", bound.min_position_,
                    bound.max_position_);
        RCLCPP_INFO(logger, "  Has velocity limits: %s",
                    bound.velocity_bounded_ ? "true" : "false");
        RCLCPP_INFO(logger, "  Max velocity: %f", bound.max_velocity_);
        RCLCPP_INFO(logger, "  Has acceleration limits: %s",
                    bound.acceleration_bounded_ ? "true" : "false");
        RCLCPP_INFO(logger, "  Max acceleration: %f", bound.max_acceleration_);
      } else {
        RCLCPP_WARN(logger, "No variable bounds found for joint %s",
                    joint_name.c_str());
      }
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_ptr = rclcpp::Node::make_shared("sample_robot_state");

  const rclcpp::Logger& logger = node_ptr->get_logger();
  robot_model_loader::RobotModelLoader robot_model_loader(node_ptr);
  const moveit::core::RobotModelPtr kinematic_model =
      robot_model_loader.getModel();

  const std::vector<moveit::core::JointModel*>& joint_models =
      kinematic_model->getJointModels();
  displayJointLimits(logger, joint_models);

  rclcpp::shutdown();
  return 0;
}
