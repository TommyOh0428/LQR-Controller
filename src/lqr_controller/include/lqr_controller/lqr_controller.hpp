#ifndef LQR_CONTROLLER__LQR_CONTROLLER_HPP_
#define LQR_CONTROLLER__LQR_CONTROLLER_HPP_

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

#include "lqr_controller/lqr_solver.hpp"

#include <Eigen/Dense>

namespace lqr_controller
{

class LQRController : public nav2_core::Controller
{
public:
  LQRController();
  ~LQRController() override;

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
  rclcpp::Logger logger_{rclcpp::get_logger("lqr_controller")};
  rclcpp::Clock::SharedPtr clock_;

  // --- Stored path ---
  nav_msgs::msg::Path global_plan_;
  size_t last_closest_idx_{0}; // forward-only tracking to prevent backward snap

  // --- Tunable parameters (declared as ROS2 params) ---
  double desired_speed_;         // v_ref constant (m/s)
  double max_linear_speed_;      // upper clamp for v (m/s)
  double max_angular_speed_;     // upper clamp for |omega| (rad/s)
  double lookahead_distance_;    // meters ahead on path for reference point
  double goal_slowdown_radius_;  // distance to goal at which speed ramps down (m)
  double dt_;                    // discretization timestep (s)
  int    lqr_horizon_;           // number of steps for receding-horizon LQR

  // Q diagonal weights (3x3 state cost)
  double q_long_;                // along-track error weight
  double q_lat_;                 // lateral error weight
  double q_head_;                // heading error weight

  // R diagonal weights (2x2 control effort cost)
  double r_v_;                   // linear velocity effort weight
  double r_omega_;               // angular velocity effort weight

  // --- LQR matrices (Eigen3) ---
  Eigen::Matrix3d Q_;                       // 3x3 state cost
  Eigen::Matrix2d R_;                       // 2x2 control cost

  // --- Helper methods ---
  size_t findClosestPoint(const geometry_msgs::msg::PoseStamped & pose);
  size_t findLookaheadPoint(size_t closest_idx);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  double computePathHeading(size_t idx);
  double computeRemainingPathLength(size_t from_idx);
  double computeReferenceSpeed(size_t from_idx, double remaining_dist);
};

}  // namespace lqr_controller

#endif
