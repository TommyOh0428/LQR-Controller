#ifndef MPPI_CONTROLLER__MPPI_SOLVER_HPP_
#define MPPI_CONTROLLER__MPPI_SOLVER_HPP_

#include <array>
#include <functional>
#include <random>
#include <vector>

namespace mppi_solver
{

// State = [x, y, theta]; Action = [v, omega]
using State = std::array<double, 3>;
using Action = std::array<double, 2>;

// Step cost callback: given next state, applied action, and step index t.
// The controller layer captures the path + costmap inside this lambda so the
// solver stays framework-free.
using StepCost = std::function<double(const State &, const Action &, int)>;

struct MPPIParams
{
  int n_samples;
  int horizon;
  double dt;

  double v_min;
  double v_max;
  double omega_max;

  double noise_sigma_v;
  double noise_sigma_omega;

  double temperature;
  double w_du;
  double epsilon;

  int seed;
};

class MPPI
{
public:
  explicit MPPI(const MPPIParams & params);

  // Reset nominal action sequence. If warm_v >= 0, seed v column with it.
  void reset(double warm_v);

  // Run one MPPI step from current state and return the first action [v, omega].
  Action get_action(const State & state, const StepCost & step_cost);

private:
  // Dubins unicycle forward step.
  State dubins_step(const State & x, const Action & u) const;

  MPPIParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  // Nominal action sequence shape (H, 2). Shifted each tick (receding horizon).
  std::vector<Action> nominal_actions_;
};

}  // namespace mppi_solver

#endif
