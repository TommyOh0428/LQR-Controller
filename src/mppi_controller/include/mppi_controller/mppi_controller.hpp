#ifndef MPPI_CONTROLLER__MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER__MPPI_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mppi_controller/mppi_solver.hpp"

namespace mppi_controller
{

class MPPIController : public nav2_core::Controller
{
public:
  MPPIController();
  ~MPPIController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // --- Node / Nav2 handles ---
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("mppi_controller")};
  rclcpp::Clock::SharedPtr clock_;

  // --- Stored path ---
  nav_msgs::msg::Path global_plan_;
  size_t last_closest_idx_{0};

  // --- Tunable parameters ---
  double desired_speed_;
  double max_linear_speed_;
  double min_linear_speed_;
  double max_angular_speed_;
  double dt_;
  int    horizon_;
  int    n_samples_;
  double noise_sigma_v_;
  double noise_sigma_omega_;
  double temperature_;
  double w_du_;
  double epsilon_;
  int    seed_;
  double goal_slowdown_radius_;

  // Cost weights
  double w_path_;
  double w_head_;
  double w_goal_;
  double w_obs_;
  double w_u_;

  // --- Solver ---
  std::unique_ptr<mppi_solver::MPPI> solver_;

  // --- Helpers (same as LQR) ---
  size_t findClosestPoint(const geometry_msgs::msg::PoseStamped & pose);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  double computePathHeading(size_t idx) const;
  double computeRemainingPathLength(size_t from_idx) const;

  // Path-following step cost used inside the MPPI rollout.
  double stepCost(
    const mppi_solver::State & x_next,
    const mppi_solver::Action & u,
    int t) const;

  // Costmap obstacle penalty for world coord (wx, wy).
  double obstaclePenalty(double wx, double wy) const;

  // Find nearest path index to (wx, wy), searching forward from last_closest_idx_.
  size_t nearestPathIdx(double wx, double wy) const;
};

}  // namespace mppi_controller

#endif
