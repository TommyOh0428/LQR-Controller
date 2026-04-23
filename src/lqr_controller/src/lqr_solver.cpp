#include "lqr_controller/lqr_solver.hpp"

#include <cmath>
#include <angles/angles.h>

namespace lqr_solver
{

void buildLinearSystem(
  double v_ref, double dt,
  Eigen::Matrix3d & A_d,
  Eigen::Matrix<double, 3, 2> & B_d)
{
  // A_d = I + A_c * dt (Forward Euler)
  // A_c has only one non-zero entry: A_c(1,2) = v_ref
  A_d = Eigen::Matrix3d::Identity();
  A_d(1, 2) = v_ref * dt;  // e_lat row, e_theta column

  // B_d = B_c * dt
  // B_c(0,0) = 1 (e_long responds to delta_v)
  // B_c(2,1) = 1 (e_theta responds to delta_omega)
  B_d = Eigen::Matrix<double, 3, 2>::Zero();
  B_d(0, 0) = dt;  // e_long responds to delta_v
  B_d(2, 1) = dt;  // e_theta responds to delta_omega
}

bool solveDARE(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & Q,
  const Eigen::Matrix2d & R,
  int max_iterations, double tolerance,
  Eigen::Matrix3d & P)
{
  // Initialize P = Q
  P = Q;

  for (int k = 0; k < max_iterations; k++) {
    // M = R + B^T * P * B  (2x2 matrix, cheap to invert)
    Eigen::Matrix2d M = R + B_d.transpose() * P * B_d;

    // P_new = Q + A^T * P * A - A^T * P * B * M^{-1} * B^T * P * A
    Eigen::Matrix3d P_new = Q + A_d.transpose() * P * A_d
      - A_d.transpose() * P * B_d * M.inverse() * B_d.transpose() * P * A_d;

    // Check convergence: max element-wise absolute difference
    double diff = (P_new - P).cwiseAbs().maxCoeff();
    P = P_new;

    if (diff < tolerance) {
      return true;  // converged
    }
  }

  return false;  // did not converge within max_iterations
}

void computeGain(
  const Eigen::Matrix3d & A_d,
  const Eigen::Matrix<double, 3, 2> & B_d,
  const Eigen::Matrix3d & P,
  const Eigen::Matrix2d & R,
  Eigen::Matrix<double, 2, 3> & K)
{
  // K = (R + B^T * P * B)^{-1} * B^T * P * A
  Eigen::Matrix2d M = R + B_d.transpose() * P * B_d;
  K = M.inverse() * B_d.transpose() * P * A_d;
}

Eigen::Vector3d computeBodyFrameError(
  double x, double y, double theta,
  double x_ref, double y_ref, double theta_ref)
{
  double dx = x - x_ref;
  double dy = y - y_ref;

  // Rotate position error into reference frame
  double e_long  =  std::cos(theta_ref) * dx + std::sin(theta_ref) * dy;  // along-track
  double e_lat   = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;  // cross-track
  double e_theta = angles::normalize_angle(theta - theta_ref);             // heading

  return Eigen::Vector3d(e_long, e_lat, e_theta);
}

}  // namespace lqr_solver
