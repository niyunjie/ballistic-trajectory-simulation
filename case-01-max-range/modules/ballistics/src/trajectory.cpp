#include "ballistics/trajectory.hpp"

#include <algorithm>
#include <cmath>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/interpolation.hpp"
#include "ballistics/rk4.hpp"

namespace ballistics {

TrajectoryResult simulateTrajectory(double thetaDeg, double psiDeg,
                                    const IntegrationConfig& config,
                                    const StepCallback& liveStepCallback,
                                    bool enableLiveCallback) {
  TrajectoryResult result;
  const auto& runtime = currentConfig();
  State state = makeInitialState(thetaDeg, psiDeg);
  double time = 0.0;
  double step = config.step;
  result.samples.push_back(makeSample(time, state));
  result.maxAltitude = state.r - runtime.earth.radius;
  result.maxSpeed = state.V;

  constexpr double maxFlightTime = 4'000.0;
  constexpr std::size_t maxAcceptedSteps = 200'000;

  while (time < maxFlightTime && result.acceptedSteps < maxAcceptedSteps) {
    State nextState;
    double acceptedStep = step;

    if (config.adaptive) {
      bool accepted = false;
      while (!accepted) {
        const State fullStep = rk4Step(state, step);
        const State halfStep = rk4Step(state, step * 0.5);
        const State refinedStep = rk4Step(halfStep, step * 0.5);
        const double error = stateErrorNorm(fullStep, refinedStep);
        if (error <= config.tolerance || step <= config.minStep * 1.01) {
          nextState = refinedStep + (refinedStep - fullStep) / 15.0;
          acceptedStep = step;
          const double ratio =
              error < 1e-12 ? 4.0 : 0.9 * std::pow(config.tolerance / error, 0.2);
          step = clamp(step * ratio, config.minStep, config.maxStep);
          accepted = true;
        } else {
          const double ratio = 0.9 * std::pow(config.tolerance / error, 0.25);
          step = clamp(step * ratio, config.minStep, config.maxStep);
        }
      }
    } else {
      nextState = rk4Step(state, step);
    }

    time += acceptedStep;
    nextState.lambda = wrapLongitude(nextState.lambda);
    const Sample nextSample = makeSample(time, nextState);
    result.samples.push_back(nextSample);
    result.acceptedSteps += 1;
    result.maxAltitude =
        std::max(result.maxAltitude, nextSample.state.r - runtime.earth.radius);
    result.maxSpeed = std::max(result.maxSpeed, nextSample.state.V);

    if (enableLiveCallback && liveStepCallback) {
      liveStepCallback(result.samples);
    }

    const double height = nextSample.state.r - runtime.earth.radius;
    if (height <= 0.0) {
      const Sample& previous = result.samples[result.samples.size() - 2];
      const double u =
          interpolateImpactFraction(previous, nextSample, config.impactInterpolation);
      result.impactTime = previous.t + (nextSample.t - previous.t) * u;
      result.impactState =
          config.impactInterpolation == ImpactInterpolation::Linear
              ? interpolateLinearState(previous, nextSample, u)
              : interpolateHermiteState(previous, nextSample, u);
      result.impactState.r = runtime.earth.radius;
      result.range = greatCircleDistance(
          toRadians(runtime.vehicle.initialLongitudeDeg),
          toRadians(runtime.vehicle.initialLatitudeDeg),
          result.impactState.lambda, result.impactState.phi);
      result.impacted = true;
      break;
    }

    state = nextState;
  }

  return result;
}

}  // namespace ballistics
