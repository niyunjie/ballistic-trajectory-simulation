#include "ballistics/search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/interpolation.hpp"
#include "ballistics/logging.hpp"
#include "ballistics/trajectory.hpp"

namespace ballistics {

namespace {

struct CartesianPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

enum class ObjectiveKind { Farthest, Boundary, Target };

struct ObjectiveContext {
  ObjectiveKind kind = ObjectiveKind::Farthest;
  double headingDeg = 0.0;
  double targetLambda = 0.0;
  double targetPhi = 0.0;
  double targetRadius = 0.0;
  CartesianPoint targetCartesian;
};

struct CandidateEvaluation {
  SearchOutcome outcome;
  double score = -std::numeric_limits<double>::infinity();
  double missDistance = std::numeric_limits<double>::infinity();
  double impactDistance = std::numeric_limits<double>::infinity();
  State closestState;
  double closestTime = 0.0;
  bool valid = false;
};

struct SearchTuning {
  int generationCount = 0;
  int refineIterations = 0;
  double mutationScale = 1.0;
  bool allowEarlyExit = false;
};

struct ClosestApproach {
  double missDistance = std::numeric_limits<double>::infinity();
  State state;
  double time = 0.0;
  bool valid = false;
};

IntegrationConfig makeFixedLinearConfig() {
  const auto& runtime = currentConfig();
  return {false,
          runtime.integration.fixedStep,
          runtime.integration.fixedStep,
          runtime.integration.fixedStep,
          0.0,
          ImpactInterpolation::Linear,
          "Fixed-step RK4 + linear interpolation"};
}

IntegrationConfig makeFixedHermiteConfig() {
  const auto& runtime = currentConfig();
  return {false,
          runtime.integration.fixedStep,
          runtime.integration.fixedStep,
          runtime.integration.fixedStep,
          0.0,
          ImpactInterpolation::CubicHermite,
          "Fixed-step RK4 + cubic Hermite interpolation"};
}

IntegrationConfig makeAdaptiveHermiteConfig() {
  const auto& runtime = currentConfig();
  return {true,
          runtime.integration.adaptiveStep,
          runtime.integration.adaptiveMinStep,
          runtime.integration.adaptiveMaxStep,
          runtime.integration.adaptiveTolerance,
          ImpactInterpolation::CubicHermite,
          "Adaptive RK4 + cubic Hermite interpolation"};
}

bool hasAnyEnabledMethod() {
  const auto& methods = currentConfig().methods;
  return methods.fixedLinear || methods.fixedHermite || methods.adaptiveHermite ||
         methods.geneticAdaptive;
}

bool hasAnyEnabledDeterministicMethod() {
  const auto& methods = currentConfig().methods;
  return methods.fixedLinear || methods.fixedHermite || methods.adaptiveHermite;
}

IntegrationConfig primaryDeterministicConfig() {
  const auto& methods = currentConfig().methods;
  if (methods.adaptiveHermite) {
    return makeAdaptiveHermiteConfig();
  }
  if (methods.fixedHermite) {
    return makeFixedHermiteConfig();
  }
  return makeFixedLinearConfig();
}

double wrapHeadingDeg(double headingDeg) {
  while (headingDeg < 0.0) {
    headingDeg += 360.0;
  }
  while (headingDeg >= 360.0) {
    headingDeg -= 360.0;
  }
  return headingDeg;
}

double initialBearingDeg(double lambda0, double phi0, double lambda1,
                         double phi1) {
  const double deltaLambda = lambda1 - lambda0;
  const double y = std::sin(deltaLambda) * std::cos(phi1);
  const double x = std::cos(phi0) * std::sin(phi1) -
                   std::sin(phi0) * std::cos(phi1) * std::cos(deltaLambda);
  return wrapHeadingDeg(toDegrees(std::atan2(y, x)));
}

CartesianPoint stateToCartesian(const State& state) {
  const double cosPhi = std::cos(state.phi);
  return {state.r * cosPhi * std::cos(state.lambda),
          state.r * cosPhi * std::sin(state.lambda),
          state.r * std::sin(state.phi)};
}

double squaredDistance(const CartesianPoint& a, const CartesianPoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

CartesianPoint lerpCartesian(const CartesianPoint& lhs, const CartesianPoint& rhs,
                             double t) {
  return {lhs.x + (rhs.x - lhs.x) * t, lhs.y + (rhs.y - lhs.y) * t,
          lhs.z + (rhs.z - lhs.z) * t};
}

State lerpState(const State& lhs, const State& rhs, double t) {
  return lhs + (rhs - lhs) * t;
}

ClosestApproach closestApproachToTarget(const TrajectoryResult& trajectory,
                                        const CartesianPoint& targetCartesian) {
  ClosestApproach best;
  auto considerPoint = [&](const State& state, double time) {
    const double distance =
        std::sqrt(squaredDistance(stateToCartesian(state), targetCartesian));
    if (!best.valid || distance < best.missDistance) {
      best.missDistance = distance;
      best.state = state;
      best.time = time;
      best.valid = true;
    }
  };

  auto considerSegment = [&](const Sample& lhs, const Sample& rhs) {
    const CartesianPoint a = stateToCartesian(lhs.state);
    const CartesianPoint b = stateToCartesian(rhs.state);
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double abz = b.z - a.z;
    const double abNorm2 = abx * abx + aby * aby + abz * abz;
    double blend = 0.0;
    if (abNorm2 > 1e-12) {
      const double apx = targetCartesian.x - a.x;
      const double apy = targetCartesian.y - a.y;
      const double apz = targetCartesian.z - a.z;
      blend = clamp((apx * abx + apy * aby + apz * abz) / abNorm2, 0.0, 1.0);
    }

    const CartesianPoint closest = lerpCartesian(a, b, blend);
    const double distance = std::sqrt(squaredDistance(closest, targetCartesian));
    if (!best.valid || distance < best.missDistance) {
      best.missDistance = distance;
      best.state = lerpState(lhs.state, rhs.state, blend);
      best.time = lhs.t + (rhs.t - lhs.t) * blend;
      best.valid = true;
    }
  };

  for (std::size_t i = 0; i < trajectory.samples.size(); ++i) {
    considerPoint(trajectory.samples[i].state, trajectory.samples[i].t);
    if (i > 0) {
      considerSegment(trajectory.samples[i - 1], trajectory.samples[i]);
    }
  }

  if (trajectory.impacted) {
    considerPoint(trajectory.impactState, trajectory.impactTime);
    if (!trajectory.samples.empty()) {
      const Sample terminalSample = trajectory.samples.back();
      const Sample impactSample = {trajectory.impactTime, trajectory.impactState};
      considerSegment(terminalSample, impactSample);
    }
  }

  return best;
}

State terminalState(const TrajectoryResult& trajectory) {
  if (trajectory.impacted) {
    return trajectory.impactState;
  }
  if (!trajectory.samples.empty()) {
    return trajectory.samples.back().state;
  }
  return {};
}

double terminalThetaDeg(const TrajectoryResult& trajectory) {
  return toDegrees(terminalState(trajectory).theta);
}

double terminalPsiDeg(const TrajectoryResult& trajectory) {
  return wrapHeadingDeg(toDegrees(terminalState(trajectory).psi));
}

double trajectoryRangeMeters(const TrajectoryResult& trajectory) {
  if (trajectory.impacted) {
    return trajectory.range;
  }
  if (trajectory.samples.empty()) {
    return 0.0;
  }
  const auto& runtime = currentConfig();
  const State state = trajectory.samples.back().state;
  return greatCircleDistance(toRadians(runtime.vehicle.initialLongitudeDeg),
                             toRadians(runtime.vehicle.initialLatitudeDeg),
                             state.lambda, state.phi);
}

double impactBearingDeg(const TrajectoryResult& trajectory) {
  const auto& runtime = currentConfig();
  const State state = terminalState(trajectory);
  return initialBearingDeg(toRadians(runtime.vehicle.initialLongitudeDeg),
                           toRadians(runtime.vehicle.initialLatitudeDeg),
                           state.lambda, state.phi);
}

double headingDifferenceDeg(double lhs, double rhs) {
  double diff = wrapHeadingDeg(lhs - rhs);
  if (diff > 180.0) {
    diff -= 360.0;
  }
  return diff;
}

ControlProfile clampControlProfile(const ControlProfile& control) {
  const auto& runtime = currentConfig();
  ControlProfile clamped = control;
  clamped.targetThetaDeg =
      clamp(clamped.targetThetaDeg, runtime.search.minThetaDeg,
            runtime.search.maxThetaDeg);
  clamped.targetPsiDeg = wrapHeadingDeg(clamped.targetPsiDeg);
  return clamped;
}

double controlRegularization(const ControlProfile& control) {
  const auto& runtime = currentConfig();
  return 0.35 * std::abs(control.targetThetaDeg - runtime.vehicle.initialThetaDeg) +
         0.08 * std::abs(headingDifferenceDeg(control.targetPsiDeg,
                                              runtime.vehicle.initialPsiDeg));
}

ControlProfile makeBaselineControlProfile() {
  ControlProfile control;
  control.targetThetaDeg = currentConfig().vehicle.initialThetaDeg;
  control.targetPsiDeg = currentConfig().vehicle.initialPsiDeg;
  return clampControlProfile(control);
}

ControlProfile randomControlProfile(std::mt19937& rng, double preferredPsiDeg) {
  const auto& runtime = currentConfig();
  ControlProfile control;
  std::uniform_real_distribution<double> thetaDist(runtime.search.minThetaDeg,
                                                   runtime.search.maxThetaDeg);
  std::uniform_real_distribution<double> psiDist(0.0, 360.0);
  control.targetThetaDeg = thetaDist(rng);
  const double randomPsi = psiDist(rng);
  control.targetPsiDeg = wrapHeadingDeg(0.45 * randomPsi + 0.55 * preferredPsiDeg);
  return clampControlProfile(control);
}

std::vector<ControlProfile> makeDirectedBoundarySeeds(double headingDeg) {
  std::vector<ControlProfile> seeds;
  seeds.reserve(5);
  const auto& runtime = currentConfig();
  const double thetaMid =
      0.5 * (runtime.search.attackThetaMinDeg + runtime.search.attackThetaMaxDeg);
  for (double thetaDeg : {runtime.search.attackThetaMinDeg, thetaMid,
                          runtime.search.attackThetaMaxDeg,
                          clamp(thetaMid + 8.0, runtime.search.minThetaDeg,
                                runtime.search.maxThetaDeg)}) {
    ControlProfile control = makeBaselineControlProfile();
    control.targetThetaDeg = thetaDeg;
    control.targetPsiDeg = headingDeg;
    seeds.push_back(clampControlProfile(control));
  }
  return seeds;
}

std::vector<ControlProfile> makeDirectedTargetSeeds(
    const ObjectiveContext& objective) {
  const auto& runtime = currentConfig();
  std::vector<ControlProfile> seeds;
  seeds.reserve(6);
  seeds.push_back(makeBaselineControlProfile());

  const double targetBearingDeg = initialBearingDeg(
      toRadians(runtime.vehicle.initialLongitudeDeg),
      toRadians(runtime.vehicle.initialLatitudeDeg), objective.targetLambda,
      objective.targetPhi);

  auto makeSeed = [&](double thetaDeg, double psiDeg) {
    ControlProfile control = makeBaselineControlProfile();
    control.targetThetaDeg = thetaDeg;
    control.targetPsiDeg = psiDeg;
    seeds.push_back(clampControlProfile(control));
  };

  const double thetaMid =
      0.5 * (runtime.search.minThetaDeg + runtime.search.maxThetaDeg);
  makeSeed(runtime.search.minThetaDeg, targetBearingDeg);
  makeSeed(thetaMid, targetBearingDeg);
  makeSeed(runtime.search.maxThetaDeg, targetBearingDeg);
  makeSeed(thetaMid, wrapHeadingDeg(targetBearingDeg - runtime.target.headingSearchHalfSpanDeg));
  makeSeed(thetaMid, wrapHeadingDeg(targetBearingDeg + runtime.target.headingSearchHalfSpanDeg));

  return seeds;
}

SearchOutcome makeSearchOutcome(const ControlProfile& control,
                                const TrajectoryResult& trajectory) {
  SearchOutcome outcome;
  outcome.control = control;
  outcome.trajectory = trajectory;
  outcome.range = trajectoryRangeMeters(trajectory);
  outcome.thetaDeg = terminalThetaDeg(trajectory);
  outcome.psiDeg = terminalPsiDeg(trajectory);
  return outcome;
}

TrajectoryResult makePartialTrajectory(const std::vector<Sample>& samples) {
  TrajectoryResult partial;
  partial.samples = samples;
  partial.acceptedSteps = samples.empty() ? 0 : samples.size() - 1;
  if (!samples.empty()) {
    partial.maxAltitude = 0.0;
    partial.maxSpeed = 0.0;
    for (const auto& sample : samples) {
      partial.maxAltitude = std::max(
          partial.maxAltitude, sample.state.r - currentConfig().earth.radius);
      partial.maxSpeed = std::max(partial.maxSpeed, sample.state.V);
    }
  }
  return partial;
}

CandidateSummary summarizeSamples(const std::vector<Sample>& samples) {
  CandidateSummary summary;
  if (samples.empty()) {
    return summary;
  }
  const auto& runtime = currentConfig();
  const State lastState = samples.back().state;
  summary.thetaDeg = toDegrees(lastState.theta);
  summary.psiDeg = wrapHeadingDeg(toDegrees(lastState.psi));
  summary.flightTime = samples.back().t;
  summary.maxAltitude = 0.0;
  for (const auto& sample : samples) {
    summary.maxAltitude =
        std::max(summary.maxAltitude, sample.state.r - runtime.earth.radius);
  }
  summary.range = greatCircleDistance(
      toRadians(runtime.vehicle.initialLongitudeDeg),
      toRadians(runtime.vehicle.initialLatitudeDeg), lastState.lambda, lastState.phi);
  summary.impactLongitudeDeg = toDegrees(lastState.lambda);
  summary.impactLatitudeDeg = toDegrees(lastState.phi);
  return summary;
}

CandidateSummary summarizeOutcome(const SearchOutcome& outcome) {
  CandidateSummary summary;
  summary.thetaDeg = outcome.thetaDeg;
  summary.psiDeg = outcome.psiDeg;
  summary.range = outcome.range;
  summary.flightTime = outcome.trajectory.impacted
                           ? outcome.trajectory.impactTime
                           : (outcome.trajectory.samples.empty()
                                  ? 0.0
                                  : outcome.trajectory.samples.back().t);
  summary.maxAltitude = outcome.trajectory.maxAltitude;
  const State state = terminalState(outcome.trajectory);
  summary.impactLongitudeDeg = toDegrees(state.lambda);
  summary.impactLatitudeDeg = toDegrees(state.phi);
  return summary;
}

StepCallback makeStreamingCallback(
    const std::string& phase, const std::string& message, double progressBase,
    double progressSpan, LiveWriter& liveWriter, const SearchOutcome& bestOutcome,
    const std::vector<BoundaryPoint>& boundary,
    const std::vector<std::vector<PlotPoint>>& attackTrajectories,
    const std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  return [=, &liveWriter](const std::vector<Sample>& samples) {
    const std::size_t acceptedSteps = samples.empty() ? 0 : samples.size() - 1;
    if (acceptedSteps == 0 ||
        acceptedSteps % static_cast<std::size_t>(
                            std::max(1, runtime.output.liveIntegrationStepStride)) !=
            0) {
      return;
    }

    const TrajectoryResult partial = makePartialTrajectory(samples);
    const double subProgress =
        1.0 - std::exp(-static_cast<double>(acceptedSteps) / 140.0);
    liveWriter.writeRunningState(
        phase, message, progressBase + progressSpan * subProgress,
        summarizeSamples(samples),
        resampleTrajectory(partial, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeOutcome(bestOutcome),
        resampleTrajectory(bestOutcome.trajectory,
                           ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, attackTrajectories, attackHeadings);
  };
}

double localEastMeters(double lambda0, double phi0, double lambda1, double phi1) {
  const auto& runtime = currentConfig();
  return (lambda1 - lambda0) * std::cos(0.5 * (phi0 + phi1)) * runtime.earth.radius;
}

double localNorthMeters(double phi0, double phi1) {
  return (phi1 - phi0) * currentConfig().earth.radius;
}

double boundaryObjectiveMeters(const TrajectoryResult& trajectory, double headingDeg) {
  if (!trajectory.impacted) {
    return -750'000.0;
  }

  const double range = trajectoryRangeMeters(trajectory);
  const double achievedHeadingDeg = impactBearingDeg(trajectory);
  const double headingErrorDeg =
      std::abs(headingDifferenceDeg(achievedHeadingDeg, headingDeg));
  const double headingErrorRad = toRadians(headingErrorDeg);
  const double directionalRange = range * std::cos(headingErrorRad);
  const double crossRange = range * std::abs(std::sin(headingErrorRad));
  const double reversePenalty =
      headingErrorDeg > 90.0 ? 350'000.0 + 4'000.0 * (headingErrorDeg - 90.0)
                             : 0.0;
  return directionalRange - 1.6 * crossRange - 3'500.0 * headingErrorDeg -
         reversePenalty;
}

CandidateEvaluation evaluateCandidate(const ControlProfile& control,
                                      const IntegrationConfig& config,
                                      const ObjectiveContext& objective,
                                      const SolverControl* solverControl,
                                      LiveWriter* liveWriter,
                                      const SearchOutcome& bestOutcome,
                                      double progressBase, double progressSpan,
                                      const std::vector<BoundaryPoint>& boundary,
                                      const std::vector<std::vector<PlotPoint>>&
                                          attackTrajectories,
                                      const std::vector<double>& attackHeadings) {
  if (solverControl != nullptr) {
    solverControl->checkpoint();
  }
  const ControlProfile clamped = clampControlProfile(control);
  const StepCallback callback =
      liveWriter == nullptr
          ? StepCallback{}
          : makeStreamingCallback(
                objective.kind == ObjectiveKind::Target ? "target_solution"
                                                        : "search_optimal",
                objective.kind == ObjectiveKind::Target
                    ? "Optimizing target intercept control profile"
                    : "Optimizing control profile",
                progressBase, progressSpan, *liveWriter, bestOutcome, boundary,
                attackTrajectories, attackHeadings);

  CandidateEvaluation evaluation;
  evaluation.outcome =
      makeSearchOutcome(clamped,
                        simulateTrajectory(clamped, config, callback,
                                           liveWriter != nullptr, solverControl));

  const double regularizationWeight =
      objective.kind == ObjectiveKind::Boundary
          ? 250.0
          : objective.kind == ObjectiveKind::Farthest ? 350.0 : 500.0;
  const double regularizationMeters =
      regularizationWeight * controlRegularization(clamped);
  if (objective.kind == ObjectiveKind::Farthest) {
    evaluation.score = evaluation.outcome.range - regularizationMeters;
    evaluation.valid = evaluation.outcome.trajectory.impacted;
    return evaluation;
  }

  if (objective.kind == ObjectiveKind::Boundary) {
    evaluation.score = boundaryObjectiveMeters(evaluation.outcome.trajectory,
                                               objective.headingDeg) -
                       regularizationMeters;
    evaluation.valid = evaluation.outcome.trajectory.impacted;
    return evaluation;
  }

  const auto& trajectory = evaluation.outcome.trajectory;
  const ClosestApproach closest =
      closestApproachToTarget(trajectory, objective.targetCartesian);
  evaluation.valid = closest.valid;
  evaluation.missDistance = closest.missDistance;
  evaluation.closestState = closest.state;
  evaluation.closestTime = closest.time;

  if (trajectory.impacted) {
    evaluation.impactDistance =
        std::sqrt(squaredDistance(stateToCartesian(trajectory.impactState),
                                  objective.targetCartesian));
  }

  const bool preferImpact = currentConfig().target.heightM <= 1.0;
  const double targetMissMeters =
      preferImpact
          ? (trajectory.impacted ? evaluation.impactDistance
                                 : evaluation.missDistance + 400'000.0)
          : evaluation.missDistance;
  const double auxiliaryMissMeters =
      std::isfinite(evaluation.impactDistance) ? evaluation.impactDistance
                                               : evaluation.missDistance;
  evaluation.score = -targetMissMeters - 0.08 * auxiliaryMissMeters -
                     120.0 * controlRegularization(clamped);
  return evaluation;
}

bool betterEvaluation(const CandidateEvaluation& lhs,
                      const CandidateEvaluation& rhs) {
  if (std::abs(lhs.score - rhs.score) > 1e-6) {
    return lhs.score > rhs.score;
  }
  return lhs.outcome.range > rhs.outcome.range;
}

void mutateProfile(ControlProfile& control, std::mt19937& rng,
                   double mutationRate, double alphaSigmaDeg,
                   double betaSigmaDeg, double heightSigmaM) {
  std::uniform_real_distribution<double> unitDist(0.0, 1.0);
  std::normal_distribution<double> thetaNoise(0.0, alphaSigmaDeg);
  std::normal_distribution<double> psiNoise(0.0, betaSigmaDeg);
  (void)heightSigmaM;

  if (unitDist(rng) < mutationRate) {
    control.targetThetaDeg += thetaNoise(rng);
  }
  if (unitDist(rng) < mutationRate) {
    control.targetPsiDeg += psiNoise(rng);
  }
  control = clampControlProfile(control);
}

ControlProfile crossoverProfile(const ControlProfile& lhs,
                                const ControlProfile& rhs, std::mt19937& rng) {
  std::uniform_real_distribution<double> unitDist(0.0, 1.0);
  ControlProfile child;
  child.targetThetaDeg =
      unitDist(rng) < 0.5 ? lhs.targetThetaDeg : rhs.targetThetaDeg;
  const double psiBlend = unitDist(rng);
  child.targetPsiDeg =
      wrapHeadingDeg(psiBlend * lhs.targetPsiDeg + (1.0 - psiBlend) * rhs.targetPsiDeg);
  return clampControlProfile(child);
}

ControlProfile refineProfileLocally(const ControlProfile& initial,
                                    const IntegrationConfig& config,
                                    const ObjectiveContext& objective,
                                    int maxIterations,
                                    const SolverControl* solverControl) {
  CandidateEvaluation best =
      evaluateCandidate(initial, config, objective, solverControl, nullptr, {}, 0.0,
                        0.0, {}, {}, {});
  const auto& runtime = currentConfig();
  double thetaStep = std::max(0.5, 0.16 *
                                     (runtime.search.maxThetaDeg -
                                      runtime.search.minThetaDeg));
  double psiStep = objective.kind == ObjectiveKind::Target
                       ? std::max(1.0, runtime.target.headingSearchHalfSpanDeg * 0.35)
                       : 14.0;

  for (int iter = 0; iter < maxIterations; ++iter) {
    if (solverControl != nullptr) {
      solverControl->checkpoint();
    }
    bool improved = false;
    for (double delta : {thetaStep, -thetaStep}) {
      ControlProfile candidate = best.outcome.control;
      candidate.targetThetaDeg += delta;
      candidate = clampControlProfile(candidate);
      const CandidateEvaluation evaluated =
          evaluateCandidate(candidate, config, objective, solverControl, nullptr, {},
                            0.0, 0.0, {}, {}, {});
      if (betterEvaluation(evaluated, best)) {
        best = evaluated;
        improved = true;
      }
    }
    for (double delta : {psiStep, -psiStep}) {
      ControlProfile candidate = best.outcome.control;
      candidate.targetPsiDeg = wrapHeadingDeg(candidate.targetPsiDeg + delta);
      candidate = clampControlProfile(candidate);
      const CandidateEvaluation evaluated =
          evaluateCandidate(candidate, config, objective, solverControl, nullptr, {},
                            0.0, 0.0, {}, {}, {});
      if (betterEvaluation(evaluated, best)) {
        best = evaluated;
        improved = true;
      }
    }

    if (!improved) {
      thetaStep *= 0.58;
      psiStep *= 0.58;
      if (thetaStep < 0.08 && psiStep < 0.3) {
        break;
      }
    }
  }

  return best.outcome.control;
}

CandidateEvaluation optimizeControl(const ObjectiveContext& objective,
                                    const IntegrationConfig& config,
                                    const std::vector<ControlProfile>& seeds,
                                    const SearchTuning& tuning,
                                    const SolverControl* solverControl,
                                    LiveWriter* liveWriter,
                                    double progressBase, double progressSpan,
                                    const std::vector<BoundaryPoint>& boundary,
                                    const std::vector<std::vector<PlotPoint>>&
                                        attackTrajectories,
                                    const std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  const std::size_t populationSize =
      static_cast<std::size_t>(std::max(
          objective.kind == ObjectiveKind::Boundary
              ? 28
              : objective.kind == ObjectiveKind::Farthest ? 24 : 12,
          runtime.search.geneticPopulation));
  const std::size_t eliteCount = static_cast<std::size_t>(std::clamp(
      runtime.search.geneticEliteCount, 2, static_cast<int>(populationSize - 1)));
  const int generationCount = std::max(
      objective.kind == ObjectiveKind::Target
          ? 18
          : objective.kind == ObjectiveKind::Boundary ? 26 : 22,
      tuning.generationCount > 0 ? tuning.generationCount
                                 : runtime.search.geneticGenerationCount);
  const int refineIterations = tuning.refineIterations > 0
                                   ? tuning.refineIterations
                                   : (objective.kind == ObjectiveKind::Target
                                          ? std::max(10, runtime.target.refineIterations)
                                          : std::max(6, runtime.search.maxRefineIterations / 2));

  std::mt19937 rng(static_cast<std::mt19937::result_type>(
      std::max(1, runtime.search.geneticRandomSeed +
                         (objective.kind == ObjectiveKind::Boundary
                              ? static_cast<int>(objective.headingDeg * 10.0)
                              : objective.kind == ObjectiveKind::Target ? 211 : 0))));

  std::vector<ControlProfile> populationProfiles;
  populationProfiles.reserve(populationSize);
  populationProfiles.push_back(makeBaselineControlProfile());
  if (objective.kind == ObjectiveKind::Boundary) {
    const auto directedSeeds = makeDirectedBoundarySeeds(objective.headingDeg);
    populationProfiles.insert(populationProfiles.end(), directedSeeds.begin(),
                              directedSeeds.end());
  }
  for (const auto& seed : seeds) {
    populationProfiles.push_back(clampControlProfile(seed));
  }

  const double targetBearingDeg =
      objective.kind == ObjectiveKind::Target
          ? initialBearingDeg(toRadians(runtime.vehicle.initialLongitudeDeg),
                              toRadians(runtime.vehicle.initialLatitudeDeg),
                              objective.targetLambda, objective.targetPhi)
          : 0.0;
  const double preferredPsiDeg =
      objective.kind == ObjectiveKind::Boundary
          ? objective.headingDeg
          : objective.kind == ObjectiveKind::Target
                ? targetBearingDeg
                : runtime.vehicle.initialPsiDeg;

  while (populationProfiles.size() < populationSize) {
    populationProfiles.push_back(randomControlProfile(rng, preferredPsiDeg));
  }

  std::vector<CandidateEvaluation> population;
  population.reserve(populationSize);
  CandidateEvaluation best;
  for (const auto& profile : populationProfiles) {
    const CandidateEvaluation evaluated = evaluateCandidate(
        profile, config, objective, solverControl, nullptr, {}, 0.0, 0.0, boundary,
        attackTrajectories, attackHeadings);
    population.push_back(evaluated);
    if (!best.valid || betterEvaluation(evaluated, best)) {
      best = evaluated;
    }
  }

  const std::string progressPhase =
      objective.kind == ObjectiveKind::Target ? "target_solution"
                                              : "search_optimal";
  const std::string progressMessage =
      objective.kind == ObjectiveKind::Target
          ? "Searching precision target control profile"
          : objective.kind == ObjectiveKind::Boundary
                ? "Tracing attack-zone boundary"
                : "Searching global farthest control profile";

  std::uniform_real_distribution<double> unitDist(0.0, 1.0);
  for (int generation = 0; generation < generationCount; ++generation) {
    if (solverControl != nullptr) {
      solverControl->checkpoint();
    }
    std::sort(population.begin(), population.end(), betterEvaluation);
    if (betterEvaluation(population.front(), best)) {
      best = population.front();
    }

    if (liveWriter != nullptr) {
      liveWriter->writeRunningState(
          progressPhase, progressMessage,
          progressBase +
              progressSpan * static_cast<double>(generation + 1) / generationCount,
          summarizeOutcome(population.front().outcome),
          resampleTrajectory(population.front().outcome.trajectory,
                             ResampleInterpolation::Linear,
                             runtime.search.currentTrajectorySamples),
          summarizeOutcome(best.outcome),
          resampleTrajectory(best.outcome.trajectory,
                             ResampleInterpolation::CubicHermite,
                             runtime.search.bestTrajectorySamples),
          boundary, attackTrajectories, attackHeadings);
    }

    if (generation + 1 >= generationCount) {
      break;
    }

    std::vector<CandidateEvaluation> nextPopulation;
    nextPopulation.reserve(populationSize);
    for (std::size_t i = 0; i < eliteCount; ++i) {
      nextPopulation.push_back(population[i]);
    }

    const double cooling =
        0.45 + 0.55 * (1.0 - static_cast<double>(generation) / generationCount);
    const double alphaSigma =
        std::max(0.15,
                 0.12 * (runtime.search.alphaMaxDeg - runtime.search.alphaMinDeg) *
                     cooling * tuning.mutationScale);
    const double betaSigma =
        std::max(0.2,
                 0.12 * (runtime.search.betaMaxDeg - runtime.search.betaMinDeg) *
                     cooling * tuning.mutationScale);
    const double heightSigma = std::max(
        150.0,
        0.12 * (runtime.search.controlHeightMaxM - runtime.search.controlHeightMinM) *
            cooling * tuning.mutationScale);
    const std::size_t breederCount =
        std::max(eliteCount + 1, populationSize / 2);
    std::uniform_int_distribution<std::size_t> breederIndex(0, breederCount - 1);
    auto pickParent = [&]() -> const CandidateEvaluation& {
      std::size_t bestIndex = breederIndex(rng);
      for (int trial = 0; trial < 2; ++trial) {
        const std::size_t challenger = breederIndex(rng);
        if (betterEvaluation(population[challenger], population[bestIndex])) {
          bestIndex = challenger;
        }
      }
      return population[bestIndex];
    };

    while (nextPopulation.size() < populationSize) {
      if (unitDist(rng) < 0.08) {
        nextPopulation.push_back(evaluateCandidate(
            randomControlProfile(rng, preferredPsiDeg), config, objective,
            solverControl, nullptr, {}, 0.0, 0.0, boundary, attackTrajectories,
            attackHeadings));
        continue;
      }

      const CandidateEvaluation& parentA = pickParent();
      const CandidateEvaluation& parentB = pickParent();
      ControlProfile child =
          crossoverProfile(parentA.outcome.control, parentB.outcome.control, rng);
      mutateProfile(child, rng, runtime.search.geneticMutationRate, alphaSigma,
                    betaSigma, heightSigma);
      nextPopulation.push_back(evaluateCandidate(
          child, config, objective, solverControl, nullptr, {}, 0.0, 0.0, boundary,
          attackTrajectories, attackHeadings));
    }

    population = std::move(nextPopulation);

    const double hitTolerance = std::max(1.0, runtime.target.toleranceMeters);
    const bool preferImpact = currentConfig().target.heightM <= 1.0;
    const bool hitSatisfied =
        preferImpact ? (std::isfinite(best.impactDistance) &&
                        best.impactDistance <= hitTolerance)
                     : best.missDistance <= hitTolerance;
    if (tuning.allowEarlyExit && objective.kind == ObjectiveKind::Target &&
        best.valid && hitSatisfied) {
      break;
    }
  }

  for (const auto& candidate : population) {
    if (betterEvaluation(candidate, best)) {
      best = candidate;
    }
  }

  const ControlProfile refined =
      refineProfileLocally(best.outcome.control, config, objective, refineIterations,
                           solverControl);
  const CandidateEvaluation refinedEvaluation =
      evaluateCandidate(refined, config, objective, solverControl, nullptr, {}, 0.0,
                        0.0, boundary, attackTrajectories, attackHeadings);
  if (betterEvaluation(refinedEvaluation, best)) {
    best = refinedEvaluation;
  }
  return best;
}

CandidateEvaluation optimizeTargetControl(
    const ObjectiveContext& objective, LiveWriter* liveWriter, double progressBase,
    double progressSpan, const std::vector<BoundaryPoint>& boundary,
    const std::vector<std::vector<PlotPoint>>& attackTrajectories,
    const std::vector<double>& attackHeadings,
    const SolverControl* solverControl,
    const std::vector<ControlProfile>& extraSeeds = {}) {
  std::vector<ControlProfile> coarseSeeds = makeDirectedTargetSeeds(objective);
  coarseSeeds.insert(coarseSeeds.end(), extraSeeds.begin(), extraSeeds.end());

  const SearchTuning coarseTuning = {
      std::max(8, currentConfig().search.geneticGenerationCount - 4),
      std::max(4, currentConfig().target.refineIterations / 2), 1.35, false};
  const CandidateEvaluation coarseBest = optimizeControl(
      objective, makeFixedHermiteConfig(), coarseSeeds, coarseTuning, solverControl,
      liveWriter,
      progressBase, progressSpan * 0.35, boundary, attackTrajectories,
      attackHeadings);

  std::vector<ControlProfile> preciseSeeds = coarseSeeds;
  preciseSeeds.push_back(coarseBest.outcome.control);
  const SearchTuning preciseTuning = {
      std::max(currentConfig().search.geneticGenerationCount,
               currentConfig().target.refineIterations + 4),
      std::max(currentConfig().target.refineIterations, 12), 0.75, true};
  return optimizeControl(objective, makeAdaptiveHermiteConfig(), preciseSeeds,
                         preciseTuning, solverControl, liveWriter,
                         progressBase + progressSpan * 0.35,
                         progressSpan * 0.65, boundary, attackTrajectories,
                         attackHeadings);
}

ComparisonRecord makeComparisonRecord(const std::string& name,
                                      const TrajectoryResult& result,
                                      ResampleInterpolation interpolation) {
  return {name,
          terminalThetaDeg(result),
          terminalPsiDeg(result),
          trajectoryRangeMeters(result),
          result.impacted ? result.impactTime
                          : (result.samples.empty() ? 0.0
                                                    : result.samples.back().t),
          result.acceptedSteps,
          result.maxAltitude,
          result.maxSpeed,
          toDegrees(terminalState(result).lambda),
          toDegrees(terminalState(result).phi),
          resampleTrajectory(result, interpolation,
                             currentConfig().search.speedCurveSamples),
          resampleTrajectory(result, interpolation,
                             currentConfig().search.finalTrajectorySamples)};
}

TargetSolution makeTargetSolution(const CandidateEvaluation& best) {
  const auto& runtime = currentConfig();
  TargetSolution solution;
  solution.enabled = runtime.target.enabled;
  if (!runtime.target.enabled) {
    return solution;
  }

  solution.targetLongitudeDeg = runtime.target.longitudeDeg;
  solution.targetLatitudeDeg = runtime.target.latitudeDeg;
  solution.targetHeightM = runtime.target.heightM;
  solution.targetRange =
      greatCircleDistance(toRadians(runtime.vehicle.initialLongitudeDeg),
                          toRadians(runtime.vehicle.initialLatitudeDeg),
                          toRadians(runtime.target.longitudeDeg),
                          toRadians(runtime.target.latitudeDeg));
  const bool preferImpact = runtime.target.heightM <= 1.0;
  const double reachDistance =
      preferImpact && best.outcome.trajectory.impacted && std::isfinite(best.impactDistance)
          ? best.impactDistance
          : best.missDistance;
  solution.reachable =
      best.valid && reachDistance <= runtime.target.toleranceMeters;
  solution.missDistance = best.missDistance;
  solution.impactDistance = best.impactDistance;
  solution.thetaDeg = best.valid ? toDegrees(best.closestState.theta) : 0.0;
  solution.psiDeg =
      best.valid ? wrapHeadingDeg(toDegrees(best.closestState.psi)) : 0.0;
  solution.control = best.outcome.control;
  solution.closestLongitudeDeg = best.valid ? toDegrees(best.closestState.lambda) : 0.0;
  solution.closestLatitudeDeg = best.valid ? toDegrees(best.closestState.phi) : 0.0;
  solution.closestHeightM =
      best.valid ? std::max(0.0, best.closestState.r - runtime.earth.radius) : 0.0;
  solution.closestTime = best.closestTime;
  solution.flightTime = best.outcome.trajectory.impactTime;
  solution.peakAltitude = best.outcome.trajectory.maxAltitude;
  solution.trajectory = best.outcome.trajectory;
  solution.trajectoryHermite = resampleTrajectory(
      best.outcome.trajectory, ResampleInterpolation::CubicHermite,
      runtime.search.finalTrajectorySamples);
  solution.trajectoryLinear = resampleTrajectory(
      best.outcome.trajectory, ResampleInterpolation::Linear,
      runtime.search.finalTrajectorySamples);
  return solution;
}

void appendBoundaryPoint(const SearchOutcome& outcome, double headingDeg,
                         std::vector<BoundaryPoint>& boundary,
                         std::vector<std::vector<PlotPoint>>& attackTrajectories,
                         std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  const State impactState = terminalState(outcome.trajectory);
  const double achievedHeadingDeg = impactBearingDeg(outcome.trajectory);
  boundary.push_back({achievedHeadingDeg,
                      outcome.thetaDeg,
                      outcome.range,
                      toDegrees(impactState.lambda),
                      toDegrees(impactState.phi),
                      std::max(18'000.0, 0.22 * outcome.trajectory.maxAltitude)});

  if ((boundary.size() - 1) % std::max(1, runtime.search.attackSampleStride) == 0) {
    attackTrajectories.push_back(resampleTrajectory(
        outcome.trajectory, ResampleInterpolation::CubicHermite,
        runtime.search.attackTrajectorySamples));
    attackHeadings.push_back(achievedHeadingDeg);
  }
}

std::vector<BoundaryPoint> finalizeBoundary(std::vector<BoundaryPoint> boundary) {
  if (boundary.empty()) {
    return boundary;
  }

  std::sort(boundary.begin(), boundary.end(),
            [](const BoundaryPoint& lhs, const BoundaryPoint& rhs) {
              return lhs.headingDeg < rhs.headingDeg;
            });

  std::vector<BoundaryPoint> unique;
  unique.reserve(boundary.size());
  for (const auto& point : boundary) {
    if (unique.empty()) {
      unique.push_back(point);
      continue;
    }

    const BoundaryPoint& previous = unique.back();
    const double headingGap =
        std::abs(headingDifferenceDeg(point.headingDeg, previous.headingDeg));
    const double lonGap = std::abs(point.longitudeDeg - previous.longitudeDeg);
    const double latGap = std::abs(point.latitudeDeg - previous.latitudeDeg);
    const double rangeGap = std::abs(point.range - previous.range);
    if (headingGap < 0.75 && lonGap < 0.01 && latGap < 0.01 && rangeGap < 2'000.0) {
      if (point.range > previous.range) {
        unique.back() = point;
      }
      continue;
    }
    unique.push_back(point);
  }

  if (unique.size() > 1) {
    const BoundaryPoint& first = unique.front();
    const BoundaryPoint& last = unique.back();
    const double wrapGap =
        std::abs(headingDifferenceDeg(first.headingDeg, last.headingDeg));
    const double lonGap = std::abs(first.longitudeDeg - last.longitudeDeg);
    const double latGap = std::abs(first.latitudeDeg - last.latitudeDeg);
    const double rangeGap = std::abs(first.range - last.range);
    if (wrapGap < 0.75 && lonGap < 0.01 && latGap < 0.01 && rangeGap < 2'000.0) {
      if (last.range > first.range) {
        unique.front() = last;
      }
      unique.pop_back();
    }
  }

  return unique;
}

std::vector<ComparisonRecord> buildComparisons(
    const SearchOutcome& optimalSearch, const TargetSolution& targetSolution) {
  const auto& runtime = currentConfig();
  const auto& methods = runtime.methods;
  const bool targetMode = runtime.target.enabled && targetSolution.enabled;
  const ControlProfile control =
      targetMode ? targetSolution.control : optimalSearch.control;

  std::vector<ComparisonRecord> comparisons;
  comparisons.reserve(4);
  const auto appendDeterministic = [&](bool enabled, const IntegrationConfig& config) {
    if (!enabled) {
      return;
    }
    const TrajectoryResult result =
        simulateTrajectory(control, config, nullptr, false, nullptr);
    const ResampleInterpolation interpolation =
        config.impactInterpolation == ImpactInterpolation::Linear
            ? ResampleInterpolation::Linear
            : ResampleInterpolation::CubicHermite;
    comparisons.push_back(makeComparisonRecord(config.label, result, interpolation));
  };

  appendDeterministic(methods.fixedLinear, makeFixedLinearConfig());
  appendDeterministic(methods.fixedHermite, makeFixedHermiteConfig());
  appendDeterministic(methods.adaptiveHermite, makeAdaptiveHermiteConfig());

  if (methods.geneticAdaptive) {
    comparisons.push_back(makeComparisonRecord(
        targetMode
            ? "Genetic control optimization + adaptive RK4 + cubic Hermite interpolation"
            : "Genetic farthest-control optimization + adaptive RK4 + cubic Hermite interpolation",
        targetMode ? targetSolution.trajectory : optimalSearch.trajectory,
        ResampleInterpolation::CubicHermite));
  }
  return comparisons;
}

}  // namespace

FinalResult computeAll(LiveWriter& liveWriter, const SolverControl* control) {
  const auto& runtime = currentConfig();
  if (!hasAnyEnabledMethod()) {
    throw std::runtime_error("At least one numerical method must be enabled.");
  }

  logInfo("search", "computeAll started.");

  const IntegrationConfig referenceConfig =
      hasAnyEnabledDeterministicMethod() ? primaryDeterministicConfig()
                                         : makeAdaptiveHermiteConfig();

  std::vector<BoundaryPoint> boundary;
  std::vector<std::vector<PlotPoint>> attackTrajectories;
  std::vector<double> attackHeadings;
  SearchOutcome optimalSearch;
  TargetSolution targetSolution;
  if (runtime.target.enabled) {
    if (control != nullptr) {
      control->checkpoint();
    }
    ObjectiveContext targetObjective;
    targetObjective.kind = ObjectiveKind::Target;
    targetObjective.targetLambda = toRadians(runtime.target.longitudeDeg);
    targetObjective.targetPhi = toRadians(runtime.target.latitudeDeg);
    targetObjective.targetRadius = runtime.earth.radius + runtime.target.heightM;
    targetObjective.targetCartesian = {
        targetObjective.targetRadius * std::cos(targetObjective.targetPhi) *
            std::cos(targetObjective.targetLambda),
        targetObjective.targetRadius * std::cos(targetObjective.targetPhi) *
            std::sin(targetObjective.targetLambda),
        targetObjective.targetRadius * std::sin(targetObjective.targetPhi),
    };

    const CandidateEvaluation targetBest = optimizeTargetControl(
        targetObjective, &liveWriter, 0.0, 1.0, boundary, attackTrajectories,
        attackHeadings, control);
    targetSolution = makeTargetSolution(targetBest);
    optimalSearch = targetBest.outcome;
  } else {
    ControlProfile previousBoundaryControl = makeBaselineControlProfile();
    const int headingCount = std::max(8, runtime.search.attackZoneHeadingCount);

    for (int i = 0; i < headingCount; ++i) {
      if (control != nullptr) {
        control->checkpoint();
      }
      const double headingDeg = i * (360.0 / headingCount);
      ObjectiveContext objective;
      objective.kind = ObjectiveKind::Boundary;
      objective.headingDeg = headingDeg;

      std::vector<ControlProfile> seeds = {previousBoundaryControl};
      if (!boundary.empty()) {
        seeds.push_back(optimalSearch.control);
      }
      const CandidateEvaluation bestForHeading = optimizeControl(
          objective, referenceConfig, seeds, {}, control, &liveWriter,
          0.6 * static_cast<double>(i) / headingCount, 0.6 / headingCount,
          boundary, attackTrajectories, attackHeadings);

      previousBoundaryControl = bestForHeading.outcome.control;
      appendBoundaryPoint(bestForHeading.outcome, headingDeg, boundary,
                          attackTrajectories, attackHeadings);
      if (optimalSearch.range < bestForHeading.outcome.range) {
        optimalSearch = bestForHeading.outcome;
      }

      liveWriter.writeRunningState(
          "attack_zone", "Tracing reachable attack-zone boundary",
          0.6 * static_cast<double>(i + 1) / headingCount,
          summarizeOutcome(bestForHeading.outcome),
          resampleTrajectory(bestForHeading.outcome.trajectory,
                             ResampleInterpolation::Linear,
                             runtime.search.currentTrajectorySamples),
          summarizeOutcome(optimalSearch),
          resampleTrajectory(optimalSearch.trajectory,
                             ResampleInterpolation::CubicHermite,
                             runtime.search.bestTrajectorySamples),
          boundary, attackTrajectories, attackHeadings);
    }

    if (runtime.methods.geneticAdaptive) {
      if (control != nullptr) {
        control->checkpoint();
      }
      ObjectiveContext farthestObjective;
      farthestObjective.kind = ObjectiveKind::Farthest;
      const CandidateEvaluation globalBest = optimizeControl(
          farthestObjective, makeAdaptiveHermiteConfig(),
          {optimalSearch.control, previousBoundaryControl}, {}, control, nullptr, 0.0,
          0.0,
          boundary, attackTrajectories, attackHeadings);
      if (globalBest.outcome.range > optimalSearch.range) {
        optimalSearch = globalBest.outcome;
      }
    }
  }

  FinalResult finalResult;
  finalResult.optimalThetaDeg = optimalSearch.thetaDeg;
  finalResult.optimalPsiDeg = optimalSearch.psiDeg;
  finalResult.optimalRange = optimalSearch.range;
  finalResult.optimalControl = optimalSearch.control;
  finalResult.optimalTrajectory = optimalSearch.trajectory;
  finalResult.attackBoundary = finalizeBoundary(boundary);
  finalResult.attackTrajectories = attackTrajectories;
  finalResult.attackTrajectoryHeadings = attackHeadings;
  finalResult.targetSolution = targetSolution;
  finalResult.comparisons = buildComparisons(optimalSearch, targetSolution);
  finalResult.optimalHermitePoints = resampleTrajectory(
      optimalSearch.trajectory, ResampleInterpolation::CubicHermite,
      runtime.search.finalTrajectorySamples);
  finalResult.optimalLinearPoints = resampleTrajectory(
      optimalSearch.trajectory, ResampleInterpolation::Linear,
      runtime.search.finalTrajectorySamples);
  finalResult.speedCurveHermite = resampleTrajectory(
      optimalSearch.trajectory, ResampleInterpolation::CubicHermite,
      runtime.search.speedCurveSamples);
  finalResult.speedCurveLinear = resampleTrajectory(
      optimalSearch.trajectory, ResampleInterpolation::Linear,
      runtime.search.speedCurveSamples);

  std::ostringstream logLine;
  logLine << "computeAll finished. rangeM=" << finalResult.optimalRange
          << ", attackBoundary=" << finalResult.attackBoundary.size()
          << ", targetEnabled=" << (runtime.target.enabled ? "true" : "false");
  logInfo("search", logLine.str());
  return finalResult;
}

}  // namespace ballistics
