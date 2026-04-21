#include "ballistics/search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/interpolation.hpp"
#include "ballistics/logging.hpp"

namespace ballistics {

namespace {

struct CartesianPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

double squaredDistance(const CartesianPoint& a, const CartesianPoint& b);

struct TargetSearchOutcome {
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  double missDistance = std::numeric_limits<double>::infinity();
  double closestTime = 0.0;
  State closestState;
  TrajectoryResult trajectory;
  bool valid = false;
};

struct GeneticCandidate {
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  SearchOutcome outcome;
};

CartesianPoint geodeticToCartesian(double longitudeDeg, double latitudeDeg,
                                   double radius) {
  const double lambda = toRadians(longitudeDeg);
  const double phi = toRadians(latitudeDeg);
  const double cosPhi = std::cos(phi);
  return {radius * cosPhi * std::cos(lambda), radius * cosPhi * std::sin(lambda),
          radius * std::sin(phi)};
}

void applyInterceptAssessment(const InterceptAssessment& assessment,
                              BoundaryPoint& point) {
  point.intercepted = assessment.intercepted;
  point.interceptorSiteIndex = assessment.siteIndex;
  point.interceptTime = assessment.interceptTime;
  point.interceptLongitudeDeg = assessment.interceptLongitudeDeg;
  point.interceptLatitudeDeg = assessment.interceptLatitudeDeg;
  point.interceptHeightM = assessment.interceptHeightM;
  point.interceptMarginM = assessment.marginM;
}

InterceptAssessment assessTrajectoryInterception(const TrajectoryResult& trajectory) {
  const auto& runtime = currentConfig();
  InterceptAssessment assessment;
  assessment.enabled = runtime.defense.enabled && !runtime.defense.sites.empty() &&
                       !trajectory.samples.empty();
  if (!assessment.enabled) {
    return assessment;
  }

  const int evaluationSamples = std::max(2, runtime.defense.evaluationSamples);
  const auto points =
      resampleTrajectory(trajectory, ResampleInterpolation::CubicHermite, evaluationSamples);
  if (points.empty()) {
    return assessment;
  }

  bool found = false;
  double bestInterceptTime = std::numeric_limits<double>::infinity();
  for (std::size_t siteIndex = 0; siteIndex < runtime.defense.sites.size(); ++siteIndex) {
    const auto& site = runtime.defense.sites[siteIndex];
    if (!site.enabled) {
      continue;
    }

    const CartesianPoint sitePoint =
        geodeticToCartesian(site.longitudeDeg, site.latitudeDeg,
                            runtime.earth.radius + site.heightM);
    for (const auto& point : points) {
      if (point.t < site.reactionDelayS) {
        continue;
      }
      if (point.height < site.minInterceptHeightM ||
          point.height > site.maxInterceptHeightM) {
        continue;
      }

      const CartesianPoint missilePoint =
          geodeticToCartesian(point.longitudeDeg, point.latitudeDeg,
                              runtime.earth.radius + point.height);
      const double slantRangeM = std::sqrt(squaredDistance(sitePoint, missilePoint));
      const double availableRangeM =
          std::min(site.maxRangeM,
                   site.interceptorSpeedMps * (point.t - site.reactionDelayS));
      if (slantRangeM <= availableRangeM + runtime.defense.interceptToleranceM &&
          (!found || point.t < bestInterceptTime)) {
        found = true;
        bestInterceptTime = point.t;
        assessment.intercepted = true;
        assessment.siteIndex = static_cast<int>(siteIndex);
        assessment.siteName = site.name;
        assessment.interceptTime = point.t;
        assessment.interceptLongitudeDeg = point.longitudeDeg;
        assessment.interceptLatitudeDeg = point.latitudeDeg;
        assessment.interceptHeightM = point.height;
        assessment.slantRangeM = slantRangeM;
        assessment.availableRangeM = availableRangeM;
        assessment.marginM = availableRangeM - slantRangeM;
      }
    }
  }

  return assessment;
}

std::vector<BoundaryPoint> filterBoundary(const std::vector<BoundaryPoint>& boundary,
                                          bool intercepted) {
  std::vector<BoundaryPoint> filtered;
  filtered.reserve(boundary.size());
  for (const auto& point : boundary) {
    if (point.intercepted == intercepted) {
      filtered.push_back(point);
    }
  }
  return filtered;
}

std::vector<BoundaryPoint> buildEffectiveBoundaryForAllHeadings(
    const IntegrationConfig& config) {
  const auto& runtime = currentConfig();
  const int headingCount = std::max(36, runtime.search.attackZoneHeadingCount);
  std::vector<BoundaryPoint> effectiveBoundary;
  effectiveBoundary.reserve(headingCount);

  for (int i = 0; i < headingCount; ++i) {
    const double headingDeg = i * (360.0 / headingCount);
    SearchOutcome bestSafe;
    bool hasSafe = false;

    for (int j = 0; j <= runtime.search.attackThetaSamples; ++j) {
      const double thetaDeg =
          runtime.search.attackThetaMinDeg +
          j * ((runtime.search.attackThetaMaxDeg - runtime.search.attackThetaMinDeg) /
               static_cast<double>(runtime.search.attackThetaSamples));
      const SearchOutcome current =
          evaluateTheta(thetaDeg, headingDeg, config, false, nullptr);
      if (!current.trajectory.impacted) {
        continue;
      }

      const InterceptAssessment assessment =
          assessTrajectoryInterception(current.trajectory);
      if (assessment.intercepted) {
        continue;
      }

      if (!hasSafe || current.range > bestSafe.range) {
        bestSafe = current;
        hasSafe = true;
      }
    }

    if (hasSafe) {
      effectiveBoundary.push_back(
          {headingDeg,
           bestSafe.thetaDeg,
           bestSafe.range,
           toDegrees(bestSafe.trajectory.impactState.lambda),
           toDegrees(bestSafe.trajectory.impactState.phi),
           std::max(18'000.0, 0.22 * bestSafe.trajectory.maxAltitude)});
    } else {
      effectiveBoundary.push_back(
          {headingDeg,
           0.0,
           0.0,
           runtime.vehicle.initialLongitudeDeg,
           runtime.vehicle.initialLatitudeDeg,
           2'000.0});
    }
  }

  return effectiveBoundary;
}

TargetSearchOutcome evaluateTargetCandidate(
    double thetaDeg, double psiDeg, const IntegrationConfig& config,
    double targetLambda, double targetPhi, double targetRadius);

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
          state.r * cosPhi * std::sin(state.lambda), state.r * std::sin(state.phi)};
}

double squaredDistance(const CartesianPoint& a, const CartesianPoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

TrajectoryResult makePartialTrajectory(const std::vector<Sample>& samples) {
  TrajectoryResult partial;
  partial.samples = samples;
  partial.acceptedSteps = samples.empty() ? 0 : samples.size() - 1;
  partial.impacted = false;
  return partial;
}

CandidateSummary summarizePartialCandidate(double thetaDeg,
                                          const std::vector<Sample>& samples) {
  CandidateSummary summary;
  summary.thetaDeg = thetaDeg;
  if (samples.empty()) {
    return summary;
  }
  const auto& runtime = currentConfig();
  summary.maxAltitude = 0.0;
  for (const auto& sample : samples) {
    summary.maxAltitude = std::max(
        summary.maxAltitude, sample.state.r - runtime.earth.radius);
  }
  summary.flightTime = samples.back().t;
  return summary;
}

StepCallback makeStreamingCallback(
    double thetaDeg, const std::string& phase, const std::string& message,
    double progressBase, double progressSpan, LiveWriter& liveWriter,
    const CandidateSummary& bestSummary,
    const std::vector<PlotPoint>& bestTrajectory,
    const std::vector<BoundaryPoint>& boundary,
    const std::vector<BoundaryPoint>& effectiveBoundary,
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
        1.0 - std::exp(-static_cast<double>(acceptedSteps) / 120.0);
    liveWriter.writeRunningState(
        phase, message, progressBase + progressSpan * subProgress,
        summarizePartialCandidate(thetaDeg, samples),
        resampleTrajectory(partial, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        bestSummary, bestTrajectory, boundary, effectiveBoundary,
        attackTrajectories, attackHeadings);
  };
}

bool betterGeneticCandidate(const GeneticCandidate& lhs,
                            const GeneticCandidate& rhs) {
  if (lhs.outcome.trajectory.impacted != rhs.outcome.trajectory.impacted) {
    return lhs.outcome.trajectory.impacted;
  }
  if (std::abs(lhs.outcome.range - rhs.outcome.range) > 1e-6) {
    return lhs.outcome.range > rhs.outcome.range;
  }
  if (std::abs(lhs.psiDeg - rhs.psiDeg) > 1e-6) {
    return lhs.psiDeg < rhs.psiDeg;
  }
  return lhs.thetaDeg < rhs.thetaDeg;
}

double clampThetaDeg(double thetaDeg) {
  const auto& runtime = currentConfig();
  return clamp(thetaDeg, runtime.search.minThetaDeg, runtime.search.maxThetaDeg);
}

double wrapPsiDeg(double psiDeg) {
  return wrapHeadingDeg(psiDeg);
}

bool betterSearchOutcome(const SearchOutcome& lhs, const SearchOutcome& rhs) {
  if (lhs.trajectory.impacted != rhs.trajectory.impacted) {
    return lhs.trajectory.impacted;
  }
  if (std::abs(lhs.range - rhs.range) > 1e-6) {
    return lhs.range > rhs.range;
  }
  if (std::abs(lhs.psiDeg - rhs.psiDeg) > 1e-6) {
    return lhs.psiDeg < rhs.psiDeg;
  }
  return lhs.thetaDeg < rhs.thetaDeg;
}

GeneticCandidate evaluateGeneticCandidate(double thetaDeg, double psiDeg,
                                          const IntegrationConfig& config) {
  GeneticCandidate candidate;
  candidate.thetaDeg = clampThetaDeg(thetaDeg);
  candidate.psiDeg = wrapPsiDeg(psiDeg);
  candidate.outcome =
      evaluateTheta(candidate.thetaDeg, candidate.psiDeg, config, false, nullptr);
  return candidate;
}

std::size_t selectTournamentWinner(const std::vector<GeneticCandidate>& population,
                                   std::mt19937& rng,
                                   std::size_t tournamentSize) {
  std::uniform_int_distribution<std::size_t> pick(0, population.size() - 1);
  std::size_t bestIndex = pick(rng);
  for (std::size_t i = 1; i < tournamentSize; ++i) {
    const std::size_t candidateIndex = pick(rng);
    if (betterGeneticCandidate(population[candidateIndex], population[bestIndex])) {
      bestIndex = candidateIndex;
    }
  }
  return bestIndex;
}

SearchOutcome searchOptimalTrajectoryGenetic(
    const IntegrationConfig& config, LiveWriter* liveWriter,
    const std::vector<BoundaryPoint>* boundary,
    const std::vector<std::vector<PlotPoint>>* attackTrajectories,
    const std::vector<double>* attackHeadings) {
  const auto& runtime = currentConfig();
  const std::size_t populationSize =
      static_cast<std::size_t>(std::max(6, runtime.search.geneticPopulation));
  const std::size_t eliteCount = static_cast<std::size_t>(
      clamp(runtime.search.geneticEliteCount, 1,
            static_cast<int>(populationSize) - 1));
  const int generationCount = std::max(1, runtime.search.geneticGenerationCount);
  const std::size_t tournamentSize = std::min<std::size_t>(4, populationSize);

  std::mt19937 rng(static_cast<std::mt19937::result_type>(
      std::max(1, runtime.search.geneticRandomSeed)));
  std::uniform_real_distribution<double> thetaDist(runtime.search.minThetaDeg,
                                                   runtime.search.maxThetaDeg);
  std::uniform_real_distribution<double> psiDist(0.0, 360.0);
  std::uniform_real_distribution<double> unitDist(0.0, 1.0);

  std::vector<GeneticCandidate> population;
  population.reserve(populationSize);
  for (std::size_t i = 0; i < populationSize; ++i) {
    double thetaDeg = 0.0;
    double psiDeg = 0.0;
    if (i < populationSize / 2) {
      const double ratio = populationSize == 1
                               ? 0.0
                               : static_cast<double>(i) /
                                     static_cast<double>(populationSize - 1);
      thetaDeg = runtime.search.minThetaDeg +
                 ratio * (runtime.search.maxThetaDeg - runtime.search.minThetaDeg);
      psiDeg = 360.0 * static_cast<double>(i) /
               static_cast<double>(std::max<std::size_t>(1, populationSize));
    } else {
      thetaDeg = thetaDist(rng);
      psiDeg = psiDist(rng);
    }
    population.push_back(evaluateGeneticCandidate(thetaDeg, psiDeg, config));
  }

  std::sort(population.begin(), population.end(), betterGeneticCandidate);
  SearchOutcome best = population.front().outcome;

  for (int generation = 0; generation < generationCount; ++generation) {
    std::sort(population.begin(), population.end(), betterGeneticCandidate);
    if (population.front().outcome.range > best.range) {
      best = population.front().outcome;
    }

    if (liveWriter != nullptr && boundary != nullptr && attackTrajectories != nullptr &&
        attackHeadings != nullptr) {
      liveWriter->writeRunningState(
          "search_genetic", "Evolving global farthest trajectory with genetic search",
          static_cast<double>(generation + 1) / generationCount,
          summarizeCandidate(population.front().outcome.thetaDeg,
                             population.front().outcome.trajectory),
          resampleTrajectory(population.front().outcome.trajectory,
                             ResampleInterpolation::Linear,
                             runtime.search.currentTrajectorySamples),
          summarizeCandidate(best.thetaDeg, best.trajectory),
          resampleTrajectory(best.trajectory,
                             ResampleInterpolation::CubicHermite,
                             runtime.search.bestTrajectorySamples),
          *boundary, filterBoundary(*boundary, false), *attackTrajectories,
          *attackHeadings);
    }

    if (generation + 1 >= generationCount) {
      break;
    }

    std::vector<GeneticCandidate> nextPopulation;
    nextPopulation.reserve(populationSize);
    for (std::size_t i = 0; i < eliteCount; ++i) {
      nextPopulation.push_back(population[i]);
    }

    const double cooling =
        0.45 + 0.55 * (1.0 - static_cast<double>(generation) / generationCount);
    const double sigmaDeg = std::max(0.05,
                                     runtime.search.geneticMutationSigmaDeg * cooling);
    std::normal_distribution<double> mutationDist(0.0, sigmaDeg);

    while (nextPopulation.size() < populationSize) {
      if (unitDist(rng) < 0.10) {
        nextPopulation.push_back(
            evaluateGeneticCandidate(thetaDist(rng), psiDist(rng), config));
        continue;
      }

      const GeneticCandidate& parentA =
          population[selectTournamentWinner(population, rng, tournamentSize)];
      const GeneticCandidate& parentB =
          population[selectTournamentWinner(population, rng, tournamentSize)];
      const double blend = unitDist(rng);
      double childThetaDeg =
          blend * parentA.thetaDeg + (1.0 - blend) * parentB.thetaDeg;
      double childPsiDeg =
          wrapPsiDeg(blend * parentA.psiDeg + (1.0 - blend) * parentB.psiDeg);
      if (unitDist(rng) < runtime.search.geneticMutationRate) {
        childThetaDeg += mutationDist(rng);
      }
      if (unitDist(rng) < runtime.search.geneticMutationRate) {
        childPsiDeg = wrapPsiDeg(childPsiDeg + mutationDist(rng) * 3.0);
      }

      nextPopulation.push_back(
          evaluateGeneticCandidate(childThetaDeg, childPsiDeg, config));
    }

    population = std::move(nextPopulation);
  }

  for (const auto& candidate : population) {
    if (betterSearchOutcome(candidate.outcome, best)) {
      best = candidate.outcome;
    }
  }
  return best;
}

ComparisonRecord makeComparisonRecord(const std::string& name, double thetaDeg,
                                      double psiDeg,
                                      const TrajectoryResult& result,
                                      ResampleInterpolation interpolation) {
  return {name,
          thetaDeg,
          psiDeg,
          result.range,
          result.impactTime,
          result.acceptedSteps,
          result.maxAltitude,
          result.maxSpeed,
          toDegrees(result.impactState.lambda),
          toDegrees(result.impactState.phi),
          resampleTrajectory(result, interpolation,
                             currentConfig().search.speedCurveSamples),
          resampleTrajectory(result, interpolation,
                             currentConfig().search.finalTrajectorySamples)};
}

TargetSolution makeTargetSolutionFromOutcome(const TargetSearchOutcome& best) {
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
  solution.reachable = best.valid &&
                       best.missDistance <= runtime.target.toleranceMeters;
  solution.effectiveReachable = solution.reachable;
  solution.missDistance =
      best.valid ? best.missDistance : std::numeric_limits<double>::infinity();
  solution.thetaDeg = best.thetaDeg;
  solution.psiDeg = best.psiDeg;
  solution.closestLongitudeDeg = toDegrees(best.closestState.lambda);
  solution.closestLatitudeDeg = toDegrees(best.closestState.phi);
  solution.closestHeightM = std::max(0.0, best.closestState.r - runtime.earth.radius);
  solution.closestTime = best.closestTime;
  solution.flightTime = best.trajectory.impactTime;
  solution.peakAltitude = best.trajectory.maxAltitude;
  if (best.valid) {
    solution.trajectoryHermite = resampleTrajectory(
        best.trajectory, ResampleInterpolation::CubicHermite,
        runtime.search.finalTrajectorySamples);
    solution.trajectoryLinear = resampleTrajectory(
        best.trajectory, ResampleInterpolation::Linear,
        runtime.search.finalTrajectorySamples);
  }
  return solution;
}

TargetSearchOutcome searchTargetSolutionGenetic(const IntegrationConfig& config) {
  const auto& runtime = currentConfig();
  TargetSearchOutcome best;
  if (!runtime.target.enabled) {
    return best;
  }

  const double targetLambda = toRadians(runtime.target.longitudeDeg);
  const double targetPhi = toRadians(runtime.target.latitudeDeg);
  const double targetRadius = runtime.earth.radius + runtime.target.heightM;
  const double centerPsiDeg = initialBearingDeg(
      toRadians(runtime.vehicle.initialLongitudeDeg),
      toRadians(runtime.vehicle.initialLatitudeDeg), targetLambda, targetPhi);

  const std::size_t populationSize =
      static_cast<std::size_t>(std::max(8, runtime.search.geneticPopulation));
  const std::size_t eliteCount = static_cast<std::size_t>(
      clamp(runtime.search.geneticEliteCount, 1,
            static_cast<int>(populationSize) - 1));
  const int generationCount = std::max(1, runtime.search.geneticGenerationCount);
  const std::size_t tournamentSize = std::min<std::size_t>(4, populationSize);

  std::mt19937 rng(static_cast<std::mt19937::result_type>(
      std::max(1, runtime.search.geneticRandomSeed + 97)));
  std::uniform_real_distribution<double> thetaDist(runtime.search.minThetaDeg,
                                                   runtime.search.maxThetaDeg);
  std::uniform_real_distribution<double> headingOffsetDist(
      -runtime.target.headingSearchHalfSpanDeg, runtime.target.headingSearchHalfSpanDeg);
  std::uniform_real_distribution<double> unitDist(0.0, 1.0);

  struct TargetGeneticCandidate {
    double thetaDeg = 0.0;
    double psiDeg = 0.0;
    TargetSearchOutcome outcome;
  };

  auto betterTargetCandidate = [](const TargetGeneticCandidate& lhs,
                                  const TargetGeneticCandidate& rhs) {
    if (std::abs(lhs.outcome.missDistance - rhs.outcome.missDistance) > 1e-6) {
      return lhs.outcome.missDistance < rhs.outcome.missDistance;
    }
    return lhs.outcome.trajectory.range > rhs.outcome.trajectory.range;
  };

  auto evaluateCandidate = [&](double thetaDeg, double psiDeg) {
    TargetGeneticCandidate candidate;
    candidate.thetaDeg = clampThetaDeg(thetaDeg);
    candidate.psiDeg = wrapHeadingDeg(psiDeg);
    candidate.outcome = evaluateTargetCandidate(candidate.thetaDeg, candidate.psiDeg,
                                                config, targetLambda, targetPhi,
                                                targetRadius);
    return candidate;
  };

  auto pickWinner = [&](const std::vector<TargetGeneticCandidate>& population) {
    std::uniform_int_distribution<std::size_t> pick(0, population.size() - 1);
    std::size_t bestIndex = pick(rng);
    for (std::size_t i = 1; i < tournamentSize; ++i) {
      const std::size_t candidateIndex = pick(rng);
      if (betterTargetCandidate(population[candidateIndex], population[bestIndex])) {
        bestIndex = candidateIndex;
      }
    }
    return bestIndex;
  };

  std::vector<TargetGeneticCandidate> population;
  population.reserve(populationSize);
  for (std::size_t i = 0; i < populationSize; ++i) {
    const double thetaDeg =
        i < populationSize / 2
            ? runtime.search.minThetaDeg +
                  (runtime.search.maxThetaDeg - runtime.search.minThetaDeg) *
                      static_cast<double>(i) /
                      static_cast<double>(std::max<std::size_t>(1, populationSize - 1))
            : thetaDist(rng);
    const double psiDeg = wrapHeadingDeg(centerPsiDeg + headingOffsetDist(rng));
    population.push_back(evaluateCandidate(thetaDeg, psiDeg));
  }

  std::sort(population.begin(), population.end(), betterTargetCandidate);
  best = population.front().outcome;

  for (int generation = 0; generation < generationCount; ++generation) {
    std::sort(population.begin(), population.end(), betterTargetCandidate);
    if (!best.valid || population.front().outcome.missDistance < best.missDistance) {
      best = population.front().outcome;
    }
    if (generation + 1 >= generationCount) {
      break;
    }

    const double cooling =
        0.45 + 0.55 * (1.0 - static_cast<double>(generation) / generationCount);
    const double sigmaThetaDeg =
        std::max(0.05, runtime.search.geneticMutationSigmaDeg * cooling);
    const double sigmaPsiDeg = std::max(
        0.10, runtime.target.headingSearchHalfSpanDeg * 0.18 * cooling);
    std::normal_distribution<double> thetaMutation(0.0, sigmaThetaDeg);
    std::normal_distribution<double> psiMutation(0.0, sigmaPsiDeg);

    std::vector<TargetGeneticCandidate> nextPopulation;
    nextPopulation.reserve(populationSize);
    for (std::size_t i = 0; i < eliteCount; ++i) {
      nextPopulation.push_back(population[i]);
    }

    while (nextPopulation.size() < populationSize) {
      if (unitDist(rng) < 0.10) {
        nextPopulation.push_back(
            evaluateCandidate(thetaDist(rng),
                              wrapHeadingDeg(centerPsiDeg + headingOffsetDist(rng))));
        continue;
      }

      const auto& parentA = population[pickWinner(population)];
      const auto& parentB = population[pickWinner(population)];
      const double blend = unitDist(rng);
      double childThetaDeg =
          blend * parentA.thetaDeg + (1.0 - blend) * parentB.thetaDeg;
      double childPsiDeg =
          wrapHeadingDeg(blend * parentA.psiDeg + (1.0 - blend) * parentB.psiDeg);

      if (unitDist(rng) < runtime.search.geneticMutationRate) {
        childThetaDeg += thetaMutation(rng);
      }
      if (unitDist(rng) < runtime.search.geneticMutationRate) {
        childPsiDeg = wrapHeadingDeg(childPsiDeg + psiMutation(rng));
      }

      nextPopulation.push_back(evaluateCandidate(childThetaDeg, childPsiDeg));
    }

    population = std::move(nextPopulation);
  }

  for (const auto& candidate : population) {
    if (!best.valid || candidate.outcome.missDistance < best.missDistance) {
      best = candidate.outcome;
    }
  }
  return best;
}

TargetSearchOutcome evaluateTargetCandidate(
    double thetaDeg, double psiDeg, const IntegrationConfig& config,
    double targetLambda, double targetPhi, double targetRadius) {
  TargetSearchOutcome outcome;
  outcome.thetaDeg = thetaDeg;
  outcome.psiDeg = wrapHeadingDeg(psiDeg);
  outcome.trajectory =
      simulateTrajectory(thetaDeg, outcome.psiDeg, config, nullptr, false);

  if (outcome.trajectory.samples.empty()) {
    return outcome;
  }

  const CartesianPoint targetPoint = {
      targetRadius * std::cos(targetPhi) * std::cos(targetLambda),
      targetRadius * std::cos(targetPhi) * std::sin(targetLambda),
      targetRadius * std::sin(targetPhi)};

  double bestDistSq = std::numeric_limits<double>::infinity();
  for (const auto& sample : outcome.trajectory.samples) {
    const double distSq = squaredDistance(stateToCartesian(sample.state), targetPoint);
    if (distSq < bestDistSq) {
      bestDistSq = distSq;
      outcome.closestState = sample.state;
      outcome.closestTime = sample.t;
    }
  }

  if (outcome.trajectory.impacted) {
    const double distSq =
        squaredDistance(stateToCartesian(outcome.trajectory.impactState), targetPoint);
    if (distSq < bestDistSq) {
      bestDistSq = distSq;
      outcome.closestState = outcome.trajectory.impactState;
      outcome.closestTime = outcome.trajectory.impactTime;
    }
  }

  outcome.missDistance = std::sqrt(bestDistSq);
  outcome.valid = true;
  return outcome;
}

}  // namespace

CandidateSummary summarizeCandidate(double thetaDeg, const TrajectoryResult& traj) {
  CandidateSummary summary;
  summary.thetaDeg = thetaDeg;
  summary.psiDeg = traj.samples.empty() ? 0.0 : toDegrees(traj.samples.front().state.psi);
  summary.range = traj.range;
  summary.flightTime = traj.impactTime;
  summary.maxAltitude = traj.maxAltitude;
  summary.impactLongitudeDeg =
      traj.impacted ? toDegrees(traj.impactState.lambda) : 0.0;
  summary.impactLatitudeDeg =
      traj.impacted ? toDegrees(traj.impactState.phi) : 0.0;
  return summary;
}

SearchOutcome evaluateTheta(double thetaDeg, double psiDeg,
                            const IntegrationConfig& config, bool enableLive,
                            const StepCallback& callback) {
  SearchOutcome outcome;
  outcome.thetaDeg = thetaDeg;
  outcome.psiDeg = wrapPsiDeg(psiDeg);
  outcome.trajectory =
      simulateTrajectory(thetaDeg, outcome.psiDeg, config, callback, enableLive);
  outcome.range = outcome.trajectory.range;
  return outcome;
}

SearchOutcome searchOptimalTheta(
    double psiDeg, const IntegrationConfig& config, LiveWriter& liveWriter,
    std::vector<BoundaryPoint>& boundary,
    std::vector<std::vector<PlotPoint>>& attackTrajectories,
    std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  const int coarseCount = runtime.search.coarseThetaCount;
  SearchOutcome best;
  std::vector<double> coarseAngles;
  coarseAngles.reserve(coarseCount + 1);
  for (int i = 0; i <= coarseCount; ++i) {
    coarseAngles.push_back(
        runtime.search.minThetaDeg +
        (runtime.search.maxThetaDeg - runtime.search.minThetaDeg) * i /
            coarseCount);
  }

  for (std::size_t i = 0; i < coarseAngles.size(); ++i) {
    const CandidateSummary bestSummaryBefore =
        best.range > 0.0 ? summarizeCandidate(best.thetaDeg, best.trajectory)
                         : CandidateSummary{};
    const std::vector<PlotPoint> bestTrajectoryBefore =
        best.range > 0.0
            ? resampleTrajectory(best.trajectory, ResampleInterpolation::CubicHermite,
                                 runtime.search.bestTrajectorySamples)
            : std::vector<PlotPoint>{};
    const StepCallback streamCallback = makeStreamingCallback(
        coarseAngles[i], "search_optimal",
        "Scanning optimal shutdown flight-path angle",
        0.5 * static_cast<double>(i) / coarseAngles.size(),
        0.5 / coarseAngles.size(), liveWriter, bestSummaryBefore,
        bestTrajectoryBefore, boundary, filterBoundary(boundary, false),
        attackTrajectories, attackHeadings);
    const SearchOutcome current =
        evaluateTheta(coarseAngles[i], psiDeg, config, true, streamCallback);
    if (current.range > best.range) {
      best = current;
    }

    liveWriter.writeRunningState(
        "search_optimal",
        "Scanning optimal shutdown flight-path angle",
        0.5 * static_cast<double>(i + 1) / coarseAngles.size(),
        summarizeCandidate(current.thetaDeg, current.trajectory),
        resampleTrajectory(current.trajectory, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeCandidate(best.thetaDeg, best.trajectory),
        resampleTrajectory(best.trajectory, ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, filterBoundary(boundary, false), attackTrajectories,
        attackHeadings);
  }

  std::size_t bestIndex = 0;
  for (std::size_t i = 0; i < coarseAngles.size(); ++i) {
    if (std::abs(coarseAngles[i] - best.thetaDeg) < 1e-9) {
      bestIndex = i;
      break;
    }
  }

  double left = coarseAngles[bestIndex == 0 ? 0 : bestIndex - 1];
  double right =
      coarseAngles[bestIndex + 1 >= coarseAngles.size() ? bestIndex : bestIndex + 1];
  if (left == right) {
    left = std::max(runtime.search.minThetaDeg, best.thetaDeg - 1.0);
    right = std::min(runtime.search.maxThetaDeg, best.thetaDeg + 1.0);
  }

  double x1 = right - kGoldenRatio * (right - left);
  double x2 = left + kGoldenRatio * (right - left);
  SearchOutcome fx1 = evaluateTheta(x1, psiDeg, config, false, nullptr);
  SearchOutcome fx2 = evaluateTheta(x2, psiDeg, config, false, nullptr);

  int iteration = 0;
  while (right - left > runtime.search.thetaToleranceDeg &&
         iteration < runtime.search.maxRefineIterations) {
    if (fx1.range > best.range) {
      best = fx1;
    }
    if (fx2.range > best.range) {
      best = fx2;
    }

    if (fx1.range < fx2.range) {
      left = x1;
      x1 = x2;
      fx1 = fx2;
      x2 = left + kGoldenRatio * (right - left);
      fx2 = evaluateTheta(x2, psiDeg, config, false, nullptr);
    } else {
      right = x2;
      x2 = x1;
      fx2 = fx1;
      x1 = right - kGoldenRatio * (right - left);
      fx1 = evaluateTheta(x1, psiDeg, config, false, nullptr);
    }

    const SearchOutcome active = fx1.range > fx2.range ? fx1 : fx2;
    liveWriter.writeRunningState(
        "search_optimal",
        "Refining optimal shutdown flight-path angle",
        0.5 +
            0.5 * static_cast<double>(iteration + 1) /
                runtime.search.maxRefineIterations,
        summarizeCandidate(active.thetaDeg, active.trajectory),
        resampleTrajectory(active.trajectory, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeCandidate(best.thetaDeg, best.trajectory),
        resampleTrajectory(best.trajectory, ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, filterBoundary(boundary, false), attackTrajectories,
        attackHeadings);

    ++iteration;
  }

  if (fx1.range > best.range) {
    best = fx1;
  }
  if (fx2.range > best.range) {
    best = fx2;
  }
  return best;
}

SearchOutcome searchOptimalThetaQuiet(double psiDeg,
                                      const IntegrationConfig& config) {
  const auto& runtime = currentConfig();
  const int coarseCount = runtime.search.coarseThetaCount;
  SearchOutcome best;
  std::vector<double> coarseAngles;
  coarseAngles.reserve(coarseCount + 1);
  for (int i = 0; i <= coarseCount; ++i) {
    coarseAngles.push_back(
        runtime.search.minThetaDeg +
        (runtime.search.maxThetaDeg - runtime.search.minThetaDeg) * i /
            coarseCount);
  }

  for (double thetaDeg : coarseAngles) {
    const SearchOutcome current =
        evaluateTheta(thetaDeg, psiDeg, config, false, nullptr);
    if (betterSearchOutcome(current, best)) {
      best = current;
    }
  }

  std::size_t bestIndex = 0;
  for (std::size_t i = 0; i < coarseAngles.size(); ++i) {
    if (std::abs(coarseAngles[i] - best.thetaDeg) < 1e-9) {
      bestIndex = i;
      break;
    }
  }

  double left = coarseAngles[bestIndex == 0 ? 0 : bestIndex - 1];
  double right =
      coarseAngles[bestIndex + 1 >= coarseAngles.size() ? bestIndex : bestIndex + 1];
  if (left == right) {
    left = std::max(runtime.search.minThetaDeg, best.thetaDeg - 1.0);
    right = std::min(runtime.search.maxThetaDeg, best.thetaDeg + 1.0);
  }

  double x1 = right - kGoldenRatio * (right - left);
  double x2 = left + kGoldenRatio * (right - left);
  SearchOutcome fx1 = evaluateTheta(x1, psiDeg, config, false, nullptr);
  SearchOutcome fx2 = evaluateTheta(x2, psiDeg, config, false, nullptr);

  int iteration = 0;
  while (right - left > runtime.search.thetaToleranceDeg &&
         iteration < runtime.search.maxRefineIterations) {
    if (betterSearchOutcome(fx1, best)) {
      best = fx1;
    }
    if (betterSearchOutcome(fx2, best)) {
      best = fx2;
    }

    if (fx1.range < fx2.range) {
      left = x1;
      x1 = x2;
      fx1 = fx2;
      x2 = left + kGoldenRatio * (right - left);
      fx2 = evaluateTheta(x2, psiDeg, config, false, nullptr);
    } else {
      right = x2;
      x2 = x1;
      fx2 = fx1;
      x1 = right - kGoldenRatio * (right - left);
      fx1 = evaluateTheta(x1, psiDeg, config, false, nullptr);
    }
    ++iteration;
  }

  if (betterSearchOutcome(fx1, best)) {
    best = fx1;
  }
  if (betterSearchOutcome(fx2, best)) {
    best = fx2;
  }
  return best;
}

SearchOutcome searchOptimalHeadingAndTheta(
    const IntegrationConfig& config, LiveWriter& liveWriter,
    std::vector<BoundaryPoint>& boundary,
    std::vector<std::vector<PlotPoint>>& attackTrajectories,
    std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  const int headingCount = std::max(36, runtime.search.attackZoneHeadingCount);
  SearchOutcome best;
  SearchOutcome bestFallback;
  bool hasFallback = false;

  for (int i = 0; i < headingCount; ++i) {
    const double headingDeg = i * (360.0 / headingCount);
    const SearchOutcome current = searchOptimalThetaQuiet(headingDeg, config);

    if (!hasFallback || betterSearchOutcome(current, bestFallback)) {
      bestFallback = current;
      hasFallback = true;
    }
    if (current.trajectory.impacted && betterSearchOutcome(current, best)) {
      best = current;
    }

    if (current.trajectory.impacted) {
      boundary.push_back({headingDeg,
                          current.thetaDeg,
                          current.range,
                          toDegrees(current.trajectory.impactState.lambda),
                          toDegrees(current.trajectory.impactState.phi),
                          std::max(18'000.0,
                                   0.22 * current.trajectory.maxAltitude)});
      applyInterceptAssessment(assessTrajectoryInterception(current.trajectory),
                               boundary.back());
      if (i % std::max(1, runtime.search.attackSampleStride) == 0) {
        attackTrajectories.push_back(resampleTrajectory(
            current.trajectory, ResampleInterpolation::CubicHermite,
            runtime.search.attackTrajectorySamples));
        attackHeadings.push_back(headingDeg);
      }
    }

    const SearchOutcome& displayOutcome =
        best.trajectory.impacted ? best : bestFallback;
    liveWriter.writeRunningState(
        "search_optimal",
        "Searching global farthest trajectory across all headings",
        static_cast<double>(i + 1) / headingCount,
        summarizeCandidate(current.thetaDeg, current.trajectory),
        resampleTrajectory(current.trajectory, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeCandidate(displayOutcome.thetaDeg, displayOutcome.trajectory),
        resampleTrajectory(displayOutcome.trajectory,
                           ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, filterBoundary(boundary, false), attackTrajectories,
        attackHeadings);
  }

  return best.trajectory.impacted ? best : bestFallback;
}

std::vector<ComparisonRecord> buildComparisons(
    const SearchOutcome& optimalSearch, const TargetSolution& targetSolution,
    const SearchOutcome* geneticSearch,
    const TargetSearchOutcome* geneticTargetOutcome) {
  const auto& runtime = currentConfig();
  const auto& methods = runtime.methods;
  const bool useTargetComparison =
      runtime.target.enabled && targetSolution.enabled &&
      targetSolution.thetaDeg >= runtime.search.minThetaDeg &&
      targetSolution.thetaDeg <= runtime.search.maxThetaDeg;
  const double referenceThetaDeg =
      useTargetComparison ? targetSolution.thetaDeg : optimalSearch.thetaDeg;
  const double referencePsiDeg =
      useTargetComparison ? targetSolution.psiDeg : optimalSearch.psiDeg;

  std::vector<ComparisonRecord> comparisons;
  comparisons.reserve(4);
  const auto appendDeterministic = [&](bool enabled, const IntegrationConfig& config) {
    if (!enabled) {
      return;
    }
    const TrajectoryResult result =
        simulateTrajectory(referenceThetaDeg, referencePsiDeg, config,
                           nullptr, false);
    const ResampleInterpolation curveInterpolation =
        config.impactInterpolation == ImpactInterpolation::Linear
            ? ResampleInterpolation::Linear
            : ResampleInterpolation::CubicHermite;
    comparisons.push_back(
        makeComparisonRecord(config.label, referenceThetaDeg, referencePsiDeg, result,
                             curveInterpolation));
  };

  appendDeterministic(methods.fixedLinear, makeFixedLinearConfig());
  appendDeterministic(methods.fixedHermite, makeFixedHermiteConfig());
  appendDeterministic(methods.adaptiveHermite, makeAdaptiveHermiteConfig());

  if (methods.geneticAdaptive && useTargetComparison && geneticTargetOutcome != nullptr) {
    comparisons.push_back(makeComparisonRecord(
        "Genetic target search + adaptive RK4 + cubic Hermite interpolation",
        geneticTargetOutcome->thetaDeg, geneticTargetOutcome->psiDeg,
        geneticTargetOutcome->trajectory,
        ResampleInterpolation::CubicHermite));
  } else if (methods.geneticAdaptive && geneticSearch != nullptr) {
    comparisons.push_back(makeComparisonRecord(
        "Genetic search + adaptive RK4 + cubic Hermite interpolation",
        geneticSearch->thetaDeg, geneticSearch->psiDeg, geneticSearch->trajectory,
        ResampleInterpolation::CubicHermite));
  }
  return comparisons;
}

void buildAttackZone(const IntegrationConfig& config, LiveWriter& liveWriter,
                     const SearchOutcome& optimalSearch,
                     std::vector<BoundaryPoint>& boundary,
                     std::vector<std::vector<PlotPoint>>& attackTrajectories,
                     std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  const int headingCount = runtime.search.attackZoneHeadingCount;
  for (int i = 0; i < headingCount; ++i) {
    const double headingDeg = i * (360.0 / headingCount);
    SearchOutcome bestForHeading;
    SearchOutcome bestFallback;
    bool hasFallback = false;
    const int localThetaCount = runtime.search.attackThetaSamples;
    for (int j = 0; j <= localThetaCount; ++j) {
      const double thetaDeg =
          runtime.search.attackThetaMinDeg +
          j * ((runtime.search.attackThetaMaxDeg -
                runtime.search.attackThetaMinDeg) /
               static_cast<double>(localThetaCount));
      const SearchOutcome current =
          evaluateTheta(thetaDeg, headingDeg, config, false, nullptr);
      if (!hasFallback || current.range > bestFallback.range) {
        bestFallback = current;
        hasFallback = true;
      }
      if (current.trajectory.impacted && current.range > bestForHeading.range) {
        bestForHeading = current;
      }
    }

    const SearchOutcome& displayOutcome =
        bestForHeading.trajectory.impacted ? bestForHeading : bestFallback;
    if (bestForHeading.trajectory.impacted) {
      boundary.push_back({headingDeg,
                          bestForHeading.thetaDeg,
                          bestForHeading.range,
                          toDegrees(bestForHeading.trajectory.impactState.lambda),
                          toDegrees(bestForHeading.trajectory.impactState.phi),
                          std::max(18'000.0,
                                   0.22 * bestForHeading.trajectory.maxAltitude)});
      applyInterceptAssessment(assessTrajectoryInterception(bestForHeading.trajectory),
                               boundary.back());
    } else {
      std::ostringstream logLine;
      logLine << "Skipping attack-zone heading " << headingDeg
              << " deg because no impacted trajectory was found.";
      logInfo("search", logLine.str());
    }

    if (bestForHeading.trajectory.impacted &&
        i % runtime.search.attackSampleStride == 0) {
      attackTrajectories.push_back(resampleTrajectory(
          bestForHeading.trajectory, ResampleInterpolation::CubicHermite,
          runtime.search.attackTrajectorySamples));
      attackHeadings.push_back(headingDeg);
    }

    liveWriter.writeRunningState(
        "attack_zone",
        "Expanding attack-zone boundary",
        static_cast<double>(i + 1) / headingCount,
        summarizeCandidate(displayOutcome.thetaDeg, displayOutcome.trajectory),
        resampleTrajectory(displayOutcome.trajectory,
                           ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeCandidate(optimalSearch.thetaDeg, optimalSearch.trajectory),
        resampleTrajectory(optimalSearch.trajectory,
                           ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, filterBoundary(boundary, false), attackTrajectories,
        attackHeadings);
  }
}

TargetSolution searchTargetSolution(
    const IntegrationConfig& config, LiveWriter& liveWriter,
    const SearchOutcome& optimalSearch,
    const std::vector<BoundaryPoint>& boundary,
    const std::vector<std::vector<PlotPoint>>& attackTrajectories,
    const std::vector<double>& attackHeadings) {
  const auto& runtime = currentConfig();
  if (!runtime.target.enabled) {
    return {};
  }

  const double targetLambda = toRadians(runtime.target.longitudeDeg);
  const double targetPhi = toRadians(runtime.target.latitudeDeg);
  const double targetRadius = runtime.earth.radius + runtime.target.heightM;
  const double centerPsiDeg = initialBearingDeg(
      toRadians(runtime.vehicle.initialLongitudeDeg),
      toRadians(runtime.vehicle.initialLatitudeDeg), targetLambda, targetPhi);

  TargetSearchOutcome best;
  const int headingCount = std::max(2, runtime.target.coarseHeadingCount);
  const int thetaCount = std::max(2, runtime.target.coarseThetaCount);
  const double headingSpan = runtime.target.headingSearchHalfSpanDeg * 2.0;
  const int totalCandidates = headingCount * thetaCount;
  int evaluated = 0;

  for (int i = 0; i < headingCount; ++i) {
    const double headingOffset =
        headingCount == 1
            ? 0.0
            : -runtime.target.headingSearchHalfSpanDeg +
                  headingSpan * static_cast<double>(i) / (headingCount - 1);
    const double psiDeg = wrapHeadingDeg(centerPsiDeg + headingOffset);
    for (int j = 0; j < thetaCount; ++j) {
      const double thetaDeg =
          runtime.search.minThetaDeg +
          (runtime.search.maxThetaDeg - runtime.search.minThetaDeg) * j /
              (thetaCount - 1);
      const TargetSearchOutcome current = evaluateTargetCandidate(
          thetaDeg, psiDeg, config, targetLambda, targetPhi, targetRadius);
      if (!best.valid || current.missDistance < best.missDistance) {
        best = current;
      }
      ++evaluated;

      liveWriter.writeRunningState(
          "target_solution", "Matching requested target coordinate",
          0.8 * static_cast<double>(evaluated) / totalCandidates,
          summarizeCandidate(current.thetaDeg, current.trajectory),
          resampleTrajectory(current.trajectory, ResampleInterpolation::Linear,
                             runtime.search.currentTrajectorySamples),
          best.valid ? summarizeCandidate(best.thetaDeg, best.trajectory)
                     : summarizeCandidate(optimalSearch.thetaDeg,
                                          optimalSearch.trajectory),
          best.valid
              ? resampleTrajectory(best.trajectory,
                                   ResampleInterpolation::CubicHermite,
                                   runtime.search.bestTrajectorySamples)
              : resampleTrajectory(optimalSearch.trajectory,
                                   ResampleInterpolation::CubicHermite,
                                   runtime.search.bestTrajectorySamples),
          boundary, filterBoundary(boundary, false), attackTrajectories,
          attackHeadings);
    }
  }

  double thetaStep =
      (runtime.search.maxThetaDeg - runtime.search.minThetaDeg) / (thetaCount - 1);
  double psiStep = headingSpan / (headingCount - 1);
  for (int iter = 0; iter < runtime.target.refineIterations; ++iter) {
    bool improved = false;
    const std::array<std::pair<double, double>, 8> offsets = {{
        {thetaStep, 0.0},
        {-thetaStep, 0.0},
        {0.0, psiStep},
        {0.0, -psiStep},
        {thetaStep, psiStep},
        {thetaStep, -psiStep},
        {-thetaStep, psiStep},
        {-thetaStep, -psiStep},
    }};

    for (const auto& [thetaOffset, psiOffset] : offsets) {
      const double thetaDeg =
          clamp(best.thetaDeg + thetaOffset, runtime.search.minThetaDeg,
                runtime.search.maxThetaDeg);
      const double psiDeg = wrapHeadingDeg(best.psiDeg + psiOffset);
      const TargetSearchOutcome candidate = evaluateTargetCandidate(
          thetaDeg, psiDeg, config, targetLambda, targetPhi, targetRadius);
      if (candidate.missDistance + 1e-6 < best.missDistance) {
        best = candidate;
        improved = true;
      }
    }

    liveWriter.writeRunningState(
        "target_solution", "Refining target-matching trajectory",
        0.8 + 0.2 * static_cast<double>(iter + 1) / runtime.target.refineIterations,
        summarizeCandidate(best.thetaDeg, best.trajectory),
        resampleTrajectory(best.trajectory, ResampleInterpolation::Linear,
                           runtime.search.currentTrajectorySamples),
        summarizeCandidate(best.thetaDeg, best.trajectory),
        resampleTrajectory(best.trajectory, ResampleInterpolation::CubicHermite,
                           runtime.search.bestTrajectorySamples),
        boundary, filterBoundary(boundary, false), attackTrajectories,
        attackHeadings);

    if (!improved) {
      thetaStep *= 0.5;
      psiStep *= 0.5;
    }
  }

  TargetSolution solution = makeTargetSolutionFromOutcome(best);
  if (solution.reachable) {
    solution.interceptAssessment = assessTrajectoryInterception(best.trajectory);
    if (solution.interceptAssessment.intercepted) {
      solution.effectiveReachable = false;
    }
  }
  return solution;
}

FinalResult computeAll(LiveWriter& liveWriter) {
  const auto& runtime = currentConfig();
  if (!hasAnyEnabledMethod()) {
    throw std::runtime_error("At least one numerical method must be enabled.");
  }

  logInfo("search", "computeAll started.");

  const bool hasDeterministicMethod = hasAnyEnabledDeterministicMethod();
  const IntegrationConfig referenceConfig =
      hasDeterministicMethod ? primaryDeterministicConfig()
                             : makeAdaptiveHermiteConfig();

  std::vector<BoundaryPoint> boundary;
  std::vector<std::vector<PlotPoint>> attackTrajectories;
  std::vector<double> attackHeadings;

  SearchOutcome optimalSearch;
  if (hasDeterministicMethod) {
    optimalSearch = searchOptimalHeadingAndTheta(referenceConfig, liveWriter, boundary,
                                                attackTrajectories, attackHeadings);
  } else {
    optimalSearch = searchOptimalTrajectoryGenetic(
        referenceConfig, &liveWriter, &boundary, &attackTrajectories,
        &attackHeadings);
  }
  {
    std::ostringstream logLine;
    logLine << "Optimal search complete. thetaDeg=" << optimalSearch.thetaDeg
            << ", psiDeg=" << optimalSearch.psiDeg
            << ", rangeM=" << optimalSearch.range
            << ", steps=" << optimalSearch.trajectory.acceptedSteps;
    logInfo("search", logLine.str());
  }
  {
    std::ostringstream logLine;
    logLine << "Global heading sweep complete. boundaryPoints=" << boundary.size()
            << ", sampledTrajectories=" << attackTrajectories.size();
    logInfo("search", logLine.str());
  }

  SearchOutcome geneticSearch;
  const SearchOutcome* geneticSearchPtr = nullptr;
  if (runtime.methods.geneticAdaptive) {
    if (hasDeterministicMethod) {
      geneticSearch = searchOptimalTrajectoryGenetic(
          makeAdaptiveHermiteConfig(), nullptr, nullptr, nullptr, nullptr);
    } else {
      geneticSearch = optimalSearch;
    }
    geneticSearchPtr = &geneticSearch;
  }

  FinalResult finalResult;
  finalResult.optimalThetaDeg = optimalSearch.thetaDeg;
  finalResult.optimalPsiDeg = optimalSearch.psiDeg;
  finalResult.optimalRange = optimalSearch.range;
  finalResult.optimalTrajectory = optimalSearch.trajectory;
  finalResult.attackBoundary = boundary;
  finalResult.effectiveAttackBoundary = buildEffectiveBoundaryForAllHeadings(referenceConfig);
  finalResult.interceptedAttackBoundary = filterBoundary(boundary, true);
  finalResult.attackTrajectories = attackTrajectories;
  finalResult.attackTrajectoryHeadings = attackHeadings;
  TargetSearchOutcome geneticTargetOutcome;
  const TargetSearchOutcome* geneticTargetPtr = nullptr;
  if (runtime.target.enabled) {
    if (hasDeterministicMethod) {
      finalResult.targetSolution = searchTargetSolution(
          referenceConfig, liveWriter, optimalSearch, boundary, attackTrajectories,
          attackHeadings);
    } else {
      geneticTargetOutcome = searchTargetSolutionGenetic(referenceConfig);
      finalResult.targetSolution = makeTargetSolutionFromOutcome(geneticTargetOutcome);
      if (finalResult.targetSolution.reachable) {
        finalResult.targetSolution.interceptAssessment =
            assessTrajectoryInterception(geneticTargetOutcome.trajectory);
        if (finalResult.targetSolution.interceptAssessment.intercepted) {
          finalResult.targetSolution.effectiveReachable = false;
        }
      }
      geneticTargetPtr = &geneticTargetOutcome;
    }
    if (runtime.methods.geneticAdaptive && hasDeterministicMethod) {
      geneticTargetOutcome = searchTargetSolutionGenetic(makeAdaptiveHermiteConfig());
      geneticTargetPtr = &geneticTargetOutcome;
    }
    std::ostringstream logLine;
    logLine << "Target solve complete. reachable="
            << (finalResult.targetSolution.reachable ? "true" : "false")
            << ", effectiveReachable="
            << (finalResult.targetSolution.effectiveReachable ? "true" : "false")
            << ", missDistanceM=" << finalResult.targetSolution.missDistance
            << ", thetaDeg=" << finalResult.targetSolution.thetaDeg
            << ", psiDeg=" << finalResult.targetSolution.psiDeg;
    logInfo("search", logLine.str());
  }
  finalResult.comparisons = buildComparisons(optimalSearch, finalResult.targetSolution,
                                             geneticSearchPtr, geneticTargetPtr);
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
  {
    std::ostringstream logLine;
    logLine << "computeAll finished. comparisons=" << finalResult.comparisons.size()
            << ", finalHermitePoints=" << finalResult.optimalHermitePoints.size()
            << ", finalLinearPoints=" << finalResult.optimalLinearPoints.size();
    logInfo("search", logLine.str());
  }
  return finalResult;
}

}  // namespace ballistics
