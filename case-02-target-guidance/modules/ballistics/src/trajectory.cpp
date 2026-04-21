#include "ballistics/trajectory.hpp"

#include <algorithm>
#include <cmath>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/interpolation.hpp"
#include "ballistics/rk4.hpp"

namespace ballistics {

namespace {

struct ControlProfileScope {
  ~ControlProfileScope() {
    clearCurrentControlProfile();
  }
};

double wrapHeadingDelta(double target, double current) {
  double diff = target - current;
  while (diff > kPi) {
    diff -= 2.0 * kPi;
  }
  while (diff < -kPi) {
    diff += 2.0 * kPi;
  }
  return diff;
}

void applyGuidanceCorrection(const ControlProfile& control, double time,
                             double step, State& state) {
  constexpr double kCorrectionDurationS = 22.0;
  constexpr double kThetaTauS = 4.0;
  constexpr double kPsiTauS = 3.5;
  if (time > kCorrectionDurationS) {
    return;
  }

  const double remaining = std::max(0.0, kCorrectionDurationS - time);
  const double effectiveStep = std::min(step, remaining);
  const double thetaBlend = 1.0 - std::exp(-effectiveStep / kThetaTauS);
  const double psiBlend = 1.0 - std::exp(-effectiveStep / kPsiTauS);
  const double targetTheta = toRadians(control.targetThetaDeg);
  const double targetPsi = toRadians(control.targetPsiDeg);

  state.theta += (targetTheta - state.theta) * thetaBlend;
  state.psi += wrapHeadingDelta(targetPsi, state.psi) * psiBlend;
}

void appendSample(TrajectoryResult& result, double time, const State& state) {
  const auto& runtime = currentConfig();
  const Sample sample = makeSample(time, state);
  result.samples.push_back(sample);
  result.acceptedSteps += 1;
  result.maxAltitude =
      std::max(result.maxAltitude, sample.state.r - runtime.earth.radius);
  result.maxSpeed = std::max(result.maxSpeed, sample.state.V);
}

bool finalizeImpact(TrajectoryResult& result, ImpactInterpolation interpolation) {
  const auto& runtime = currentConfig();
  const Sample& nextSample = result.samples.back();
  const double height = nextSample.state.r - runtime.earth.radius;
  if (height > 0.0) {
    return false;
  }

  const Sample& previous = result.samples[result.samples.size() - 2];
  const double u = interpolateImpactFraction(previous, nextSample, interpolation);
  result.impactTime = previous.t + (nextSample.t - previous.t) * u;
  result.impactState =
      interpolation == ImpactInterpolation::Linear
          ? interpolateLinearState(previous, nextSample, u)
          : interpolateHermiteState(previous, nextSample, u);
  result.impactState.r = runtime.earth.radius;
  result.range = greatCircleDistance(
      toRadians(runtime.vehicle.initialLongitudeDeg),
      toRadians(runtime.vehicle.initialLatitudeDeg), result.impactState.lambda,
      result.impactState.phi);
  result.impacted = true;
  return true;
}

}  // namespace

TrajectoryResult simulateTrajectory(const ControlProfile& control,
                                    const IntegrationConfig& config,
                                    const StepCallback& liveStepCallback,
                                    bool enableLiveCallback,
                                    const SolverControl* solverControl) {
  TrajectoryResult result;
  const auto& runtime = currentConfig();
  setCurrentControlProfile(control);
  ControlProfileScope controlScope;

  State state = makeInitialState(runtime.vehicle.initialThetaDeg,
                                 runtime.vehicle.initialPsiDeg);
  double time = 0.0;
  double step = config.step;
  result.samples.push_back(makeSample(time, state));
  result.maxAltitude = state.r - runtime.earth.radius;
  result.maxSpeed = state.V;

  constexpr double maxFlightTime = 4'000.0;
  constexpr std::size_t maxAcceptedSteps = 200'000;

  while (time < maxFlightTime && result.acceptedSteps < maxAcceptedSteps) {
    if (solverControl != nullptr) {
      solverControl->checkpoint();
    }
    State nextState;
    double acceptedStep = step;

    if (config.adaptive) {
      bool accepted = false;
      while (!accepted) {
        if (solverControl != nullptr) {
          solverControl->checkpoint();
        }
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
    applyGuidanceCorrection(control, time, acceptedStep, nextState);
    appendSample(result, time, nextState);

    if (enableLiveCallback && liveStepCallback) {
      liveStepCallback(result.samples);
    }

    if (finalizeImpact(result, config.impactInterpolation)) {
      break;
    }

    state = nextState;
  }
  return result;
}

}  // namespace ballistics
