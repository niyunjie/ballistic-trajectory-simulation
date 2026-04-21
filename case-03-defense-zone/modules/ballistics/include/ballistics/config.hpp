#pragma once

#include <string>
#include <vector>

namespace ballistics {

enum class AlphaScheduleMode : int {
  ConstantBelow30km = 0,
  SmoothGlide = 1,
  QianStyle = 2,
  AggressiveSkip = 3,
  QianSkipGlide = 4,
  SangerStyle = 5,
};

struct EarthConfig {
  double radius = 6'371'000.0;
  double mu = 3.986e14;
  double omega = 7.292115e-5;
};

struct VehicleConfig {
  double mass = 500.0;
  double refArea = 0.3;
  double initialSpeed = 4000.0;
  double initialThetaDeg = 10.0;
  double initialPsiDeg = 0.0;
  double initialAlphaDeg = 0.0;
  AlphaScheduleMode alphaScheduleMode = AlphaScheduleMode::SmoothGlide;
  double initialBetaDeg = 0.0;
  double initialLongitudeDeg = 20.0;
  double initialLatitudeDeg = 38.0;
  double initialHeight = 50'000.0;
};

struct MethodSelectionConfig {
  bool fixedLinear = true;
  bool fixedHermite = true;
  bool adaptiveHermite = true;
  bool geneticAdaptive = true;
};

struct SearchConfig {
  double minThetaDeg = 1.0;
  double maxThetaDeg = 80.0;
  double thetaToleranceDeg = 1e-4;
  int coarseThetaCount = 28;
  int maxRefineIterations = 28;
  int geneticPopulation = 18;
  int geneticEliteCount = 4;
  int geneticGenerationCount = 14;
  int geneticRandomSeed = 20250401;
  double geneticMutationRate = 0.30;
  double geneticMutationSigmaDeg = 2.5;
  int attackZoneHeadingCount = 24;
  double attackThetaMinDeg = 4.0;
  double attackThetaMaxDeg = 46.0;
  int attackThetaSamples = 18;
  int attackSampleStride = 2;
  int currentTrajectorySamples = 100;
  int bestTrajectorySamples = 160;
  int finalTrajectorySamples = 260;
  int speedCurveSamples = 220;
  int attackTrajectorySamples = 140;
};

struct IntegrationDefaults {
  double fixedStep = 0.5;
  double adaptiveStep = 1.0;
  double adaptiveMinStep = 0.02;
  double adaptiveMaxStep = 2.0;
  double adaptiveTolerance = 2e-6;
};

struct TargetConfig {
  bool enabled = false;
  double longitudeDeg = 0.0;
  double latitudeDeg = 0.0;
  double heightM = 0.0;
  double toleranceMeters = 5'000.0;
  double headingSearchHalfSpanDeg = 12.0;
  int coarseHeadingCount = 17;
  int coarseThetaCount = 21;
  int refineIterations = 16;
};

struct DefenseSiteConfig {
  bool enabled = true;
  std::string name;
  double longitudeDeg = 0.0;
  double latitudeDeg = 0.0;
  double heightM = 0.0;
  double reactionDelayS = 120.0;
  double interceptorSpeedMps = 3'500.0;
  double maxRangeM = 1'200'000.0;
  double minInterceptHeightM = 20'000.0;
  double maxInterceptHeightM = 800'000.0;
};

struct DefenseConfig {
  bool enabled = false;
  int evaluationSamples = 180;
  double interceptToleranceM = 25'000.0;
  std::vector<DefenseSiteConfig> sites;
};

struct OutputConfig {
  std::wstring livePath;
  std::wstring finalPath;
  double liveDelayMs = 18.0;
  int liveIntegrationStepStride = 32;
};

struct SolverConfig {
  EarthConfig earth;
  VehicleConfig vehicle;
  MethodSelectionConfig methods;
  SearchConfig search;
  IntegrationDefaults integration;
  TargetConfig target;
  DefenseConfig defense;
  OutputConfig output;
};

void setCurrentConfig(const SolverConfig& config);
const SolverConfig& currentConfig();

}  // namespace ballistics
