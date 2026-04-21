#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ballistics {

struct State {
  double V = 0.0;
  double theta = 0.0;
  double psi = 0.0;
  double r = 0.0;
  double lambda = 0.0;
  double phi = 0.0;
};

inline State operator+(const State& a, const State& b) {
  return {a.V + b.V, a.theta + b.theta, a.psi + b.psi, a.r + b.r,
          a.lambda + b.lambda, a.phi + b.phi};
}

inline State operator-(const State& a, const State& b) {
  return {a.V - b.V, a.theta - b.theta, a.psi - b.psi, a.r - b.r,
          a.lambda - b.lambda, a.phi - b.phi};
}

inline State operator*(const State& state, double scale) {
  return {state.V * scale, state.theta * scale, state.psi * scale,
          state.r * scale, state.lambda * scale, state.phi * scale};
}

inline State operator*(double scale, const State& state) {
  return state * scale;
}

inline State operator/(const State& state, double scale) {
  return state * (1.0 / scale);
}

struct Atmosphere {
  double density = 0.0;
  double temperature = 0.0;
  double soundSpeed = 0.0;
  double mach = 0.0;
};

struct AeroCoefficients {
  double cd = 0.0;
  double cl = 0.0;
  double cy = 0.0;
  double drag = 0.0;
  double lift = 0.0;
  double sideForce = 0.0;
  double gravity = 0.0;
};

struct Evaluation {
  State derivative;
  Atmosphere atmosphere;
  AeroCoefficients aero;
};

struct Sample {
  double t = 0.0;
  State state;
  State derivative;
  Atmosphere atmosphere;
  AeroCoefficients aero;
};

enum class ImpactInterpolation {
  Linear,
  CubicHermite,
};

enum class ResampleInterpolation {
  Linear,
  CubicHermite,
};

struct IntegrationConfig {
  bool adaptive = false;
  double step = 0.5;
  double minStep = 0.02;
  double maxStep = 2.0;
  double tolerance = 1e-6;
  ImpactInterpolation impactInterpolation = ImpactInterpolation::Linear;
  std::string label;
};

struct ControlProfile {
  static constexpr std::size_t kAlphaNodeCount = 4;
  static constexpr std::size_t kBetaNodeCount = 5;

  std::array<double, kAlphaNodeCount> alphaNodesDeg = {0.0, 0.0, 0.0, 0.0};
  std::array<double, kBetaNodeCount> betaNodesDeg = {0.0, 0.0, 0.0, 0.0, 0.0};
  double transitionHeightM = 30'000.0;
  double targetThetaDeg = 10.0;
  double targetPsiDeg = 0.0;
};

struct TrajectoryResult {
  std::vector<Sample> samples;
  State impactState;
  double impactTime = 0.0;
  double range = 0.0;
  double maxAltitude = 0.0;
  double maxSpeed = 0.0;
  std::size_t acceptedSteps = 0;
  bool impacted = false;
};

struct PlotPoint {
  double t = 0.0;
  double longitudeDeg = 0.0;
  double latitudeDeg = 0.0;
  double height = 0.0;
  double speed = 0.0;
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
};

struct ComparisonRecord {
  std::string name;
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  double range = 0.0;
  double flightTime = 0.0;
  std::size_t steps = 0;
  double peakAltitude = 0.0;
  double maxSpeed = 0.0;
  double impactLongitudeDeg = 0.0;
  double impactLatitudeDeg = 0.0;
  std::vector<PlotPoint> speedCurve;
  std::vector<PlotPoint> trajectory;
};

struct BoundaryPoint {
  double headingDeg = 0.0;
  double thetaDeg = 0.0;
  double range = 0.0;
  double longitudeDeg = 0.0;
  double latitudeDeg = 0.0;
  double topHeight = 0.0;
};

struct CandidateSummary {
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  double range = 0.0;
  double flightTime = 0.0;
  double maxAltitude = 0.0;
  double impactLongitudeDeg = 0.0;
  double impactLatitudeDeg = 0.0;
};

struct SearchOutcome {
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  double range = -1.0;
  ControlProfile control;
  TrajectoryResult trajectory;
};

struct TargetSolution {
  bool enabled = false;
  bool reachable = false;
  double targetLongitudeDeg = 0.0;
  double targetLatitudeDeg = 0.0;
  double targetHeightM = 0.0;
  double targetRange = 0.0;
  double missDistance = 0.0;
  double impactDistance = 0.0;
  double thetaDeg = 0.0;
  double psiDeg = 0.0;
  ControlProfile control;
  double closestLongitudeDeg = 0.0;
  double closestLatitudeDeg = 0.0;
  double closestHeightM = 0.0;
  double closestTime = 0.0;
  double flightTime = 0.0;
  double peakAltitude = 0.0;
  TrajectoryResult trajectory;
  std::vector<PlotPoint> trajectoryHermite;
  std::vector<PlotPoint> trajectoryLinear;
};

struct FinalResult {
  double optimalThetaDeg = 0.0;
  double optimalPsiDeg = 0.0;
  double optimalRange = 0.0;
  ControlProfile optimalControl;
  TrajectoryResult optimalTrajectory;
  std::vector<ComparisonRecord> comparisons;
  std::vector<PlotPoint> optimalHermitePoints;
  std::vector<PlotPoint> optimalLinearPoints;
  std::vector<PlotPoint> speedCurveHermite;
  std::vector<PlotPoint> speedCurveLinear;
  std::vector<BoundaryPoint> attackBoundary;
  std::vector<std::vector<PlotPoint>> attackTrajectories;
  std::vector<double> attackTrajectoryHeadings;
  TargetSolution targetSolution;
};

enum class SolverStatus : std::uint8_t {
  Idle,
  Running,
  Paused,
  Done,
  Cancelled,
  Failed,
};

struct LiveState {
  SolverStatus status = SolverStatus::Idle;
  std::string phase;
  std::string message;
  double progress = 0.0;
  CandidateSummary current;
  CandidateSummary best;
  std::vector<PlotPoint> currentTrajectory;
  std::vector<PlotPoint> bestTrajectory;
  std::vector<BoundaryPoint> attackBoundary;
  std::vector<std::vector<PlotPoint>> attackTrajectories;
  std::vector<double> attackTrajectoryHeadings;
  bool hasFinalResult = false;
  FinalResult finalResult;
  std::string errorMessage;
};

}  // namespace ballistics
