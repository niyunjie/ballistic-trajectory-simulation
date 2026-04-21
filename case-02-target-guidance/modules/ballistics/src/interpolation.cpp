#include "ballistics/interpolation.hpp"

#include <algorithm>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"

namespace ballistics {

Sample makeSample(double time, const State& state) {
  const Evaluation eval = evaluateState(state);
  return {time, state, eval.derivative, eval.atmosphere, eval.aero};
}

State interpolateLinearState(const Sample& a, const Sample& b, double u) {
  return a.state + (b.state - a.state) * u;
}

double hermiteScalar(double y0, double y1, double m0, double m1, double dt,
                     double u) {
  const double u2 = u * u;
  const double u3 = u2 * u;
  return (2.0 * u3 - 3.0 * u2 + 1.0) * y0 +
         (u3 - 2.0 * u2 + u) * dt * m0 +
         (-2.0 * u3 + 3.0 * u2) * y1 +
         (u3 - u2) * dt * m1;
}

State interpolateHermiteState(const Sample& a, const Sample& b, double u) {
  const double dt = b.t - a.t;
  double values[6] = {};
  for (int i = 0; i < 6; ++i) {
    values[i] = hermiteScalar(componentAt(a.state, i), componentAt(b.state, i),
                              componentAt(a.derivative, i),
                              componentAt(b.derivative, i), dt, u);
  }
  State state = stateFromComponents(values);
  state.lambda = wrapLongitude(state.lambda);
  return state;
}

double interpolateImpactFraction(const Sample& a, const Sample& b,
                                 ImpactInterpolation interpolation) {
  const auto& config = currentConfig();
  const double h0 = a.state.r - config.earth.radius;
  const double h1 = b.state.r - config.earth.radius;
  if (interpolation == ImpactInterpolation::Linear) {
    return clamp(h0 / (h0 - h1), 0.0, 1.0);
  }

  double lo = 0.0;
  double hi = 1.0;
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double heightMid =
        interpolateHermiteState(a, b, mid).r - config.earth.radius;
    if (heightMid > 0.0) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

State interpolateTrajectoryState(const std::vector<Sample>& samples, double time,
                                 ResampleInterpolation interpolation) {
  if (samples.empty()) {
    return {};
  }
  if (time <= samples.front().t) {
    return samples.front().state;
  }
  if (time >= samples.back().t) {
    return samples.back().state;
  }

  auto upper =
      std::lower_bound(samples.begin(), samples.end(), time,
                       [](const Sample& sample, double targetTime) {
                         return sample.t < targetTime;
                       });
  if (upper == samples.begin()) {
    return upper->state;
  }
  const Sample& b = *upper;
  const Sample& a = *(upper - 1);
  const double u = (time - a.t) / (b.t - a.t);
  return interpolation == ResampleInterpolation::Linear
             ? interpolateLinearState(a, b, u)
             : interpolateHermiteState(a, b, u);
}

std::vector<PlotPoint> resampleTrajectory(const TrajectoryResult& trajectory,
                                          ResampleInterpolation interpolation,
                                          int pointCount) {
  std::vector<PlotPoint> points;
  if (trajectory.samples.empty() || pointCount <= 0) {
    return points;
  }
  const auto& config = currentConfig();
  points.reserve(static_cast<std::size_t>(pointCount));
  const double finalTime = trajectory.impacted ? trajectory.impactTime
                                               : trajectory.samples.back().t;
  for (int i = 0; i < pointCount; ++i) {
    const double u =
        pointCount == 1 ? 0.0 : static_cast<double>(i) / (pointCount - 1);
    const double time = finalTime * u;
    State state =
        interpolateTrajectoryState(trajectory.samples, time, interpolation);
    if (trajectory.impacted && i == pointCount - 1) {
      state = trajectory.impactState;
    }
    points.push_back({time,
                      toDegrees(state.lambda),
                      toDegrees(state.phi),
                      std::max(0.0, state.r - config.earth.radius),
                      state.V,
                      toDegrees(state.theta),
                      toDegrees(state.psi)});
  }
  return points;
}

}  // namespace ballistics
