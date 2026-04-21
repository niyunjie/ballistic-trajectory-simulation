#include "ballistics/rk4.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"

namespace ballistics {

State rk4Step(const State& state, double dt) {
  const State k1 = evaluateState(state).derivative;
  const State k2 = evaluateState(state + k1 * (dt * 0.5)).derivative;
  const State k3 = evaluateState(state + k2 * (dt * 0.5)).derivative;
  const State k4 = evaluateState(state + k3 * dt).derivative;
  return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

double stateErrorNorm(const State& a, const State& b) {
  const auto& config = currentConfig();
  const std::array<double, 6> scale = {4000.0, 1.0, 1.0, config.earth.radius,
                                       1.0, 1.0};
  double maxError = 0.0;
  for (int i = 0; i < 6; ++i) {
    maxError = std::max(
        maxError, std::abs(componentAt(a, i) - componentAt(b, i)) / scale[i]);
  }
  return maxError;
}

}  // namespace ballistics
