#ifndef LQR_CONTROLLER__LQR_SOLVER_HPP_
#define LQR_CONTROLLER__LQR_SOLVER_HPP_

#include <vector>
#include <Eigen/Dense>

namespace lqr_solver
{

// Solve finite-horizon LQR via backward Riccati recursion.
// A_refs[t], B_refs[t] are the linearized system at each horizon step t=0..N-1.
// Q_f is the terminal cost matrix (often same as Q).
// Returns K_0: the gain to apply at the current timestep (receding horizon).
//
// Backward pass:
//   P_N = Q_f
//   P_t = Q + A_t^T P_{t+1} A_t - A_t^T P_{t+1} B_t (R + B_t^T P_{t+1} B_t)^{-1} B_t^T P_{t+1} A_t
//   K_t = (R + B_t^T P_{t+1} B_t)^{-1} B_t^T P_{t+1} A_t
//
// Returns false if horizon is empty (caller should use zero command).
bool solveRecedingHorizonLQR(
  const std::vector<Eigen::Matrix3d> & A_refs,
  const std::vector<Eigen::Matrix<double, 3, 2>> & B_refs,
  const Eigen::Matrix3d & Q,
  const Eigen::Matrix2d & R,
  const Eigen::Matrix3d & Q_f,
  Eigen::Matrix<double, 2, 3> & K_0);

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
