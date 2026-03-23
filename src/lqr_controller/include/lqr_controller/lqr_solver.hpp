#ifndef LQR_CONTROLLER__LQR_SOLVER_HPP_
#define LQR_CONTROLLER__LQR_SOLVER_HPP_

#include <Eigen/Dense>

namespace lqr_solver
{

// Build discrete A_d, B_d matrices from unicycle linearization
// Forward Euler discretization with constant v_ref, omega_ref = 0
//
// A_d = I + A_c * dt = [1, 0, 0         ]    B_d = B_c * dt = [dt, 0 ]
//                      [0, 1, v_ref * dt ]                     [ 0, 0 ]
//                      [0, 0, 1          ]                     [ 0, dt]
void buildLinearSystem(
  double v_ref, double dt,
  Eigen::Matrix3d & A_d,
  Eigen::Matrix<double, 3, 2> & B_d);

// Solve Discrete Algebraic Riccati Equation (DARE) iteratively.
// P_{k+1} = Q + A^T P_k A - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A
// Stores result in P. Returns true if converged within max_iterations.
bool solveDARE(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & Q,
  const Eigen::Matrix2d & R,
  int max_iterations, double tolerance,
  Eigen::Matrix3d & P);

// Compute LQR gain K from DARE solution P
// K = (R + B^T P B)^{-1} B^T P A
void computeGain(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & P,
  const Eigen::Matrix2d & R,
  Eigen::Matrix<double, 2, 3> & K);

// Compute body-frame tracking error between robot pose and reference pose
// Returns [e_long, e_lat, e_theta]
//   e_long  =  cos(theta_ref)*dx + sin(theta_ref)*dy
//   e_lat   = -sin(theta_ref)*dx + cos(theta_ref)*dy
//   e_theta = normalize(theta - theta_ref)
Eigen::Vector3d computeBodyFrameError(
  double x, double y, double theta,
  double x_ref, double y_ref, double theta_ref);

// Wrap angle to [-pi, pi]
double normalizeAngle(double angle);

}  // namespace lqr_solver

#endif
