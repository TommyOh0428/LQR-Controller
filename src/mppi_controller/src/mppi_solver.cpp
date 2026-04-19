#include "mppi_controller/mppi_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mppi_solver
{

MPPI::MPPI(const MPPIParams & params)
: params_(params),
  rng_(static_cast<uint64_t>(params.seed))
{
  nominal_actions_.assign(params_.horizon, {0.0, 0.0});
}

void MPPI::reset(double warm_v)
{
  for (auto & a : nominal_actions_) {
    a[0] = (warm_v >= 0.0) ? warm_v : 0.0;
    a[1] = 0.0;
  }
}

State MPPI::dubins_step(const State & x, const Action & u) const
{
  State x_next;
  x_next[0] = x[0] + u[0] * std::cos(x[2]) * params_.dt;
  x_next[1] = x[1] + u[0] * std::sin(x[2]) * params_.dt;
  x_next[2] = x[2] + u[1] * params_.dt;
  return x_next;
}

Action MPPI::get_action(const State & state, const StepCost & step_cost)
{
  const int K = params_.n_samples;
  const int H = params_.horizon;

  // (K, H, 2) flat storage: perturbed actions that we clip and roll out.
  std::vector<std::vector<Action>> perturbed(K, std::vector<Action>(H));
  std::vector<double> total_costs(K, 0.0);

  // 1) sample noise + add to nominal + clip
  for (int k = 0; k < K; k++) {
    for (int t = 0; t < H; t++) {
      double nv = normal_(rng_) * params_.noise_sigma_v;
      double nw = normal_(rng_) * params_.noise_sigma_omega;
      double v = nominal_actions_[t][0] + nv;
      double w = nominal_actions_[t][1] + nw;
      v = std::clamp(v, params_.v_min, params_.v_max);
      w = std::clamp(w, -params_.omega_max, params_.omega_max);
      perturbed[k][t] = {v, w};
    }
  }

  // 2) rollout each sample, accumulating cost (with smoothness term for t>=1)
  for (int k = 0; k < K; k++) {
    State x = state;
    Action u_prev = {0.0, 0.0};
    double sum_c = 0.0;
    for (int t = 0; t < H; t++) {
      const Action & u = perturbed[k][t];
      x = dubins_step(x, u);
      double c = step_cost(x, u, t);
      if (t >= 1) {
        double dv = u[0] - u_prev[0];
        double dw = u[1] - u_prev[1];
        c += params_.w_du * (dv * dv + dw * dw);
      }
      sum_c += c;
      u_prev = u;
    }
    total_costs[k] = sum_c;
  }

  // 3) softmax weights with min-cost baseline subtraction
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < K; k++) {
    if (total_costs[k] < min_cost) { min_cost = total_costs[k]; }
  }
  std::vector<double> weights(K, 0.0);
  double w_sum = 0.0;
  for (int k = 0; k < K; k++) {
    double norm_c = total_costs[k] - min_cost;
    weights[k] = std::exp(-norm_c / params_.temperature);
    w_sum += weights[k];
  }
  w_sum += params_.epsilon;

  // 4) weighted average update of whole sequence
  std::vector<Action> new_actions(H, {0.0, 0.0});
  for (int t = 0; t < H; t++) {
    double sv = 0.0, sw = 0.0;
    for (int k = 0; k < K; k++) {
      sv += weights[k] * perturbed[k][t][0];
      sw += weights[k] * perturbed[k][t][1];
    }
    double v = sv / w_sum;
    double w = sw / w_sum;
    v = std::clamp(v, params_.v_min, params_.v_max);
    w = std::clamp(w, -params_.omega_max, params_.omega_max);
    new_actions[t] = {v, w};
  }

  Action best = new_actions[0];

  // 5) receding-horizon shift: nominal <- shift(new, -1), duplicate last
  for (int t = 0; t + 1 < H; t++) {
    nominal_actions_[t] = new_actions[t + 1];
  }
  if (H >= 2) {
    nominal_actions_[H - 1] = new_actions[H - 1];
  } else {
    nominal_actions_[0] = new_actions[0];
  }

  return best;
}

}  // namespace mppi_solver
