#include "lqr_controller/lqr_controller.hpp"

#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lqr_controller
{

LQRController::LQRController() = default;
LQRController::~LQRController() = default;

void LQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
}

void LQRController::cleanup()
{
}

void LQRController::activate()
{
}

void LQRController::deactivate()
{
}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  return cmd;
}

void LQRController::setPlan(const nav_msgs::msg::Path &)
{
}

void LQRController::setSpeedLimit(const double &, const bool &)
{
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, nav2_core::Controller)