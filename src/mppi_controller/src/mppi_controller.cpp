#include "mppi_controller/mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace mppi_controller
{

MPPIController::MPPIController() = default;
MPPIController::~MPPIController() = default;

void MPPIController::configure(
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
  node->declare_parameter(plugin_name_ + ".min_linear_speed", -0.05);
  node->declare_parameter(plugin_name_ + ".max_angular_speed", 2.84);
  node->declare_parameter(plugin_name_ + ".dt", 0.05);
  node->declare_parameter(plugin_name_ + ".horizon", 20);
  node->declare_parameter(plugin_name_ + ".n_samples", 500);
  node->declare_parameter(plugin_name_ + ".noise_sigma_v", 0.2);
  node->declare_parameter(plugin_name_ + ".noise_sigma_omega", 0.4);
  node->declare_parameter(plugin_name_ + ".temperature", 0.3);
  node->declare_parameter(plugin_name_ + ".w_du", 0.1);
  node->declare_parameter(plugin_name_ + ".epsilon", 1e-9);
  node->declare_parameter(plugin_name_ + ".seed", 7);
  node->declare_parameter(plugin_name_ + ".goal_slowdown_radius", 1.0);
  node->declare_parameter(plugin_name_ + ".w_path", 8.0);
  node->declare_parameter(plugin_name_ + ".w_head", 1.0);
  node->declare_parameter(plugin_name_ + ".w_goal", 2.0);
  node->declare_parameter(plugin_name_ + ".w_obs", 30.0);
  node->declare_parameter(plugin_name_ + ".w_u", 0.01);

  node->get_parameter(plugin_name_ + ".desired_speed", desired_speed_);
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".horizon", horizon_);
  node->get_parameter(plugin_name_ + ".n_samples", n_samples_);
  node->get_parameter(plugin_name_ + ".noise_sigma_v", noise_sigma_v_);
  node->get_parameter(plugin_name_ + ".noise_sigma_omega", noise_sigma_omega_);
  node->get_parameter(plugin_name_ + ".temperature", temperature_);
  node->get_parameter(plugin_name_ + ".w_du", w_du_);
  node->get_parameter(plugin_name_ + ".epsilon", epsilon_);
  node->get_parameter(plugin_name_ + ".seed", seed_);
  node->get_parameter(plugin_name_ + ".goal_slowdown_radius", goal_slowdown_radius_);
  node->get_parameter(plugin_name_ + ".w_path", w_path_);
  node->get_parameter(plugin_name_ + ".w_head", w_head_);
  node->get_parameter(plugin_name_ + ".w_goal", w_goal_);
  node->get_parameter(plugin_name_ + ".w_obs", w_obs_);
  node->get_parameter(plugin_name_ + ".w_u", w_u_);

  mppi_solver::MPPIParams sp;
  sp.n_samples = n_samples_;
  sp.horizon = horizon_;
  sp.dt = dt_;
  sp.v_min = min_linear_speed_;
  sp.v_max = max_linear_speed_;
  sp.omega_max = max_angular_speed_;
  sp.noise_sigma_v = noise_sigma_v_;
  sp.noise_sigma_omega = noise_sigma_omega_;
  sp.temperature = temperature_;
  sp.w_du = w_du_;
  sp.epsilon = epsilon_;
  sp.seed = seed_;

  solver_ = std::make_unique<mppi_solver::MPPI>(sp);
  solver_->reset(desired_speed_);

  RCLCPP_INFO(logger_, "MPPI configured: H=%d, K=%d, dt=%.3f, v_ref=%.2f",
    horizon_, n_samples_, dt_, desired_speed_);
}

void MPPIController::cleanup()
{
  RCLCPP_INFO(logger_, "%s cleaned up", plugin_name_.c_str());
}

void MPPIController::activate()
{
  RCLCPP_INFO(logger_, "%s activated", plugin_name_.c_str());
}

void MPPIController::deactivate()
{
  RCLCPP_INFO(logger_, "%s deactivated", plugin_name_.c_str());
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_closest_idx_ = 0;
  if (solver_) {
    solver_->reset(desired_speed_);
  }
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Empty path");
  }

  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose goal_pose = global_plan_.poses.back().pose;
    if (goal_checker->isGoalReached(pose.pose, goal_pose,
        geometry_msgs::msg::Twist()))
    {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = pose.header;
      return cmd;
    }
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = getYawFromQuaternion(pose.pose.orientation);

  last_closest_idx_ = findClosestPoint(pose);
  double remaining_dist = computeRemainingPathLength(last_closest_idx_);

  // Build a step-cost closure that captures `this` for access to costmap + path.
  auto cost_fn = [this](const mppi_solver::State & xn,
                        const mppi_solver::Action & u, int t) -> double {
    return this->stepCost(xn, u, t);
  };

  mppi_solver::State s0 = {x, y, theta};
  mppi_solver::Action u = solver_->get_action(s0, cost_fn);

  double v = u[0];
  double omega = u[1];

  // Goal slowdown (same as LQR)
  double speed_scale = 1.0;
  if (goal_slowdown_radius_ > 0.0 && remaining_dist < goal_slowdown_radius_) {
    speed_scale = remaining_dist / goal_slowdown_radius_;
    speed_scale = std::clamp(speed_scale, 0.05, 1.0);
  }
  v = v * speed_scale;

  v = std::clamp(v, 0.0, max_linear_speed_);
  omega = std::clamp(omega, -max_angular_speed_, max_angular_speed_);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = omega;
  return cmd;
}

double MPPIController::stepCost(
  const mppi_solver::State & x_next,
  const mppi_solver::Action & u,
  int t) const
{
  double px = x_next[0];
  double py = x_next[1];
  double th = x_next[2];

  // path distance + heading: nearest point on stored plan ahead of last_closest_idx_
  size_t idx = nearestPathIdx(px, py);
  double dx = global_plan_.poses[idx].pose.position.x - px;
  double dy = global_plan_.poses[idx].pose.position.y - py;
  double path_err_sq = dx * dx + dy * dy;

  double path_heading = computePathHeading(idx);
  double dth = th - path_heading;
  // wrap to [-pi, pi]
  while (dth > M_PI)  { dth -= 2.0 * M_PI; }
  while (dth < -M_PI) { dth += 2.0 * M_PI; }
  double head_err_sq = dth * dth;

  // goal cost (only on final horizon step)
  double goal_err_sq = 0.0;
  if (t == horizon_ - 1) {
    const auto & gp = global_plan_.poses.back().pose.position;
    double gx = gp.x - px;
    double gy = gp.y - py;
    goal_err_sq = gx * gx + gy * gy;
  }

  double obs = obstaclePenalty(px, py);
  double u_sq = u[0] * u[0] + u[1] * u[1];

  return w_path_ * path_err_sq
       + w_head_ * head_err_sq
       + w_goal_ * goal_err_sq
       + w_obs_ * obs
       + w_u_ * u_sq;
}

double MPPIController::obstaclePenalty(double wx, double wy) const
{
  if (!costmap_ros_) { return 0.0; }
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) { return 0.0; }

  // world -> map coord conversion
  unsigned int mx = 0, my = 0;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return 1e3;  // out of bounds
  }
  unsigned char c = costmap->getCost(mx, my);
  if (c == nav2_costmap_2d::LETHAL_OBSTACLE ||
      c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return 1e4;
  }
  return static_cast<double>(c) / 252.0;
}

size_t MPPIController::nearestPathIdx(double wx, double wy) const
{
  size_t best = last_closest_idx_;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = last_closest_idx_; i < global_plan_.poses.size(); i++) {
    double dx = global_plan_.poses[i].pose.position.x - wx;
    double dy = global_plan_.poses[i].pose.position.y - wy;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

size_t MPPIController::findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose)
{
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;

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

double MPPIController::getYawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double MPPIController::computeRemainingPathLength(size_t from_idx) const
{
  double length = 0.0;
  for (size_t i = from_idx; i + 1 < global_plan_.poses.size(); i++) {
    double dx = global_plan_.poses[i + 1].pose.position.x
      - global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y
      - global_plan_.poses[i].pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

double MPPIController::computePathHeading(size_t idx) const
{
  const auto & q = global_plan_.poses[idx].pose.orientation;

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

void MPPIController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_linear_speed_ = desired_speed_ * speed_limit / 100.0;
  } else {
    max_linear_speed_ = speed_limit;
  }
}

}  // namespace mppi_controller

PLUGINLIB_EXPORT_CLASS(mppi_controller::MPPIController, nav2_core::Controller)
