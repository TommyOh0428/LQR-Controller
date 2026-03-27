#include "lqr_controller/lqr_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace lqr_controller
{

LQRController::LQRController() = default;
LQRController::~LQRController() = default;

void LQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  node->declare_parameter(plugin_name_ + ".desired_speed", 0.2);
  node->declare_parameter(plugin_name_ + ".max_linear_speed", 0.22);
  node->declare_parameter(plugin_name_ + ".max_angular_speed", 2.84);
  node->declare_parameter(plugin_name_ + ".lookahead_distance", 0.5);
  node->declare_parameter(plugin_name_ + ".dt", 0.05);
  node->declare_parameter(plugin_name_ + ".dare_max_iterations", 1000);
  node->declare_parameter(plugin_name_ + ".dare_tolerance", 1e-9);
  node->declare_parameter(plugin_name_ + ".q_longitudinal", 1.0);
  node->declare_parameter(plugin_name_ + ".q_lateral", 3.0);
  node->declare_parameter(plugin_name_ + ".q_heading", 1.0);
  node->declare_parameter(plugin_name_ + ".r_v", 1.0);
  node->declare_parameter(plugin_name_ + ".r_omega", 0.5);

  node->get_parameter(plugin_name_ + ".desired_speed", desired_speed_);
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".dare_max_iterations", dare_max_iterations_);
  node->get_parameter(plugin_name_ + ".dare_tolerance", dare_tolerance_);
  node->get_parameter(plugin_name_ + ".q_longitudinal", q_long_);
  node->get_parameter(plugin_name_ + ".q_lateral", q_lat_);
  node->get_parameter(plugin_name_ + ".q_heading", q_head_);
  node->get_parameter(plugin_name_ + ".r_v", r_v_);
  node->get_parameter(plugin_name_ + ".r_omega", r_omega_);

  Q_ = Eigen::Matrix3d::Zero();
  Q_(0, 0) = q_long_;
  Q_(1, 1) = q_lat_;
  Q_(2, 2) = q_head_;

  R_ = Eigen::Matrix2d::Zero();
  R_(0, 0) = r_v_;
  R_(1, 1) = r_omega_;

  lqr_solver::buildLinearSystem(desired_speed_, dt_, A_d_, B_d_);

  bool converged = lqr_solver::solveDARE(
    A_d_, B_d_, Q_, R_, dare_max_iterations_, dare_tolerance_, P_);
  if (!converged) {
    RCLCPP_WARN(logger_, "DARE did not converge within %d iterations", dare_max_iterations_);
  }

  lqr_solver::computeGain(A_d_, B_d_, P_, R_, K_);

  RCLCPP_INFO(logger_, "LQR gain K computed:");
  RCLCPP_INFO(logger_, "  K = [%f, %f, %f]", K_(0, 0), K_(0, 1), K_(0, 2));
  RCLCPP_INFO(logger_, "      [%f, %f, %f]", K_(1, 0), K_(1, 1), K_(1, 2));
}

void LQRController::cleanup()
{
  RCLCPP_INFO(logger_, "%s cleaned up", plugin_name_.c_str());
}

void LQRController::activate()
{
  RCLCPP_INFO(logger_, "%s activated", plugin_name_.c_str());
}

void LQRController::deactivate()
{
  RCLCPP_INFO(logger_, "%s deactivated", plugin_name_.c_str());
}

void LQRController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_closest_idx_ = 0;
}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Empty path");
  }

  size_t closest_idx = findClosestPoint(pose);
  size_t ref_idx = findLookaheadPoint(closest_idx);
  const auto & ref_pose = global_plan_.poses[ref_idx];

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = getYawFromQuaternion(pose.pose.orientation);

  double x_ref = ref_pose.pose.position.x;
  double y_ref = ref_pose.pose.position.y;
  double theta_ref = computePathHeading(ref_idx);

  Eigen::Vector3d error = lqr_solver::computeBodyFrameError(
    x, y, theta, x_ref, y_ref, theta_ref);

  Eigen::Vector2d delta_u = -K_ * error;

  double v = desired_speed_ + delta_u(0);
  double omega = 0.0 + delta_u(1);

  v = std::clamp(v, 0.0, max_linear_speed_);
  omega = std::clamp(omega, -max_angular_speed_, max_angular_speed_);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = omega;

  return cmd;
}

size_t LQRController::findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose)
{
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;

  // Search forward only from last known position to prevent backward snap.
  // Small lookback window (2 indices) handles localization jitter.
  size_t search_start = (last_closest_idx_ > 2) ? last_closest_idx_ - 2 : 0;

  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = last_closest_idx_;

  for (size_t i = search_start; i < global_plan_.poses.size(); i++) {
    double dx = global_plan_.poses[i].pose.position.x - robot_x;
    double dy = global_plan_.poses[i].pose.position.y - robot_y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  last_closest_idx_ = closest_idx;
  return closest_idx;
}

size_t LQRController::findLookaheadPoint(size_t closest_idx)
{
  if (global_plan_.poses.size() < 2) {
    return closest_idx;
  }

  double accumulated_dist = 0.0;

  for (size_t i = closest_idx; i < global_plan_.poses.size() - 1; i++) {
    double dx = global_plan_.poses[i + 1].pose.position.x
      - global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y
      - global_plan_.poses[i].pose.position.y;
    accumulated_dist += std::sqrt(dx * dx + dy * dy);

    if (accumulated_dist >= lookahead_distance_) {
      return i + 1;
    }
  }

  return global_plan_.poses.size() - 1;
}

double LQRController::getYawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double LQRController::computePathHeading(size_t idx)
{
  const auto & q = global_plan_.poses[idx].pose.orientation;

  // Detect identity quaternion — NavFn doesn't set heading on path poses.
  // Fall back to geometric direction between consecutive waypoints.
  bool is_identity = (std::abs(q.w - 1.0) < 1e-3 &&
                      std::abs(q.x) < 1e-3 &&
                      std::abs(q.y) < 1e-3 &&
                      std::abs(q.z) < 1e-3);

  if (!is_identity) {
    return getYawFromQuaternion(q);
  }

  if (idx + 1 < global_plan_.poses.size()) {
    double dx = global_plan_.poses[idx + 1].pose.position.x
      - global_plan_.poses[idx].pose.position.x;
    double dy = global_plan_.poses[idx + 1].pose.position.y
      - global_plan_.poses[idx].pose.position.y;
    return std::atan2(dy, dx);
  }
  if (idx > 0) {
    double dx = global_plan_.poses[idx].pose.position.x
      - global_plan_.poses[idx - 1].pose.position.x;
    double dy = global_plan_.poses[idx].pose.position.y
      - global_plan_.poses[idx - 1].pose.position.y;
    return std::atan2(dy, dx);
  }
  return 0.0;
}

void LQRController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_linear_speed_ = desired_speed_ * speed_limit / 100.0;
  } else {
    max_linear_speed_ = speed_limit;
  }
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, nav2_core::Controller)
