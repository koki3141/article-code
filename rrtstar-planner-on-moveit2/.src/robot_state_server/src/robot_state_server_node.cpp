#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "robot_state_interface/srv/joint_limit.hpp"

class RobotStateServerNode : public rclcpp::Node {
 public:
  RobotStateServerNode() : Node("robot_state_server_node") {
    this->get_logger().set_level(rclcpp::Logger::Level::Info);

    joint_limit_server_ =
        this->create_service<robot_state_interface::srv::JointLimit>(
            "joint_limits",
            std::bind(&RobotStateServerNode::getJointLimits, this,
                      std::placeholders::_1, std::placeholders::_2));
  }

  void initialize() {
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    kinematic_model_ = robot_model_loader.getModel();
  }

 private:
  void getJointLimits(
      const std::shared_ptr<robot_state_interface::srv::JointLimit::Request>,
      std::shared_ptr<robot_state_interface::srv::JointLimit::Response>
          response) {
    RCLCPP_DEBUG(this->get_logger(), "call getJointLimits");
    const std::vector<moveit::core::JointModel*>& joint_models =
        kinematic_model_->getJointModels();
    for (const auto& joint : joint_models) {
      if (joint->getVariableCount() > 0) {
        auto bounds = joint->getVariableBounds();
        response->name.push_back(joint->getName());
        response->min_position.push_back(bounds[0].min_position_);
        response->max_position.push_back(bounds[0].max_position_);
      }
    }
  }
  rclcpp::Service<robot_state_interface::srv::JointLimit>::SharedPtr
      joint_limit_server_;

  moveit::core::RobotModelPtr kinematic_model_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto robot_server_state_node = std::make_shared<RobotStateServerNode>();
  robot_server_state_node->initialize();

  rclcpp::spin(robot_server_state_node);

  rclcpp::shutdown();
  return 0;
}
