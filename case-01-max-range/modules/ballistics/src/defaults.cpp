#include "ballistics/defaults.hpp"

namespace ballistics {

SolverConfig makeDefaultConfig() {
  SolverConfig config;

  config.earth.radius = 6'371'000.0;
  config.earth.mu = 3.986e14;
  config.earth.omega = 7.292115e-5;

  config.vehicle.mass = 500.0;
  config.vehicle.refArea = 0.3;
  config.vehicle.initialSpeed = 4000.0;
  config.vehicle.initialThetaDeg = 10.0;
  config.vehicle.initialPsiDeg = 0.0;
  config.vehicle.initialAlphaDeg = 8.5;
  config.vehicle.alphaScheduleMode = AlphaScheduleMode::QianStyle;
  config.vehicle.initialBetaDeg = 0.0;
  config.vehicle.initialLongitudeDeg = 20.0;
  config.vehicle.initialLatitudeDeg = 38.0;
  config.vehicle.initialHeight = 50'000.0;

  config.methods.fixedLinear = true;
  config.methods.fixedHermite = true;
  config.methods.adaptiveHermite = true;
  config.methods.geneticAdaptive = true;

  config.search.minThetaDeg = 1.0;
  config.search.maxThetaDeg = 80.0;
  config.search.thetaToleranceDeg = 1e-4;
  config.search.coarseThetaCount = 20;
  config.search.maxRefineIterations = 20;
  config.search.geneticPopulation = 18;
  config.search.geneticEliteCount = 4;
  config.search.geneticGenerationCount = 14;
  config.search.geneticRandomSeed = 20250401;
  config.search.geneticMutationRate = 0.30;
  config.search.geneticMutationSigmaDeg = 2.5;
  config.search.attackZoneHeadingCount = 16;
  config.search.attackThetaMinDeg = 4.0;
  config.search.attackThetaMaxDeg = 46.0;
  config.search.attackThetaSamples = 12;
  config.search.attackSampleStride = 4;
  config.search.currentTrajectorySamples = 72;
  config.search.bestTrajectorySamples = 120;
  config.search.finalTrajectorySamples = 180;
  config.search.speedCurveSamples = 180;
  config.search.attackTrajectorySamples = 80;

  config.integration.fixedStep = 0.5;
  config.integration.adaptiveStep = 1.0;
  config.integration.adaptiveMinStep = 0.02;
  config.integration.adaptiveMaxStep = 2.0;
  config.integration.adaptiveTolerance = 2e-6;

  config.target.enabled = true;
  config.target.longitudeDeg = 32.0;
  config.target.latitudeDeg = 43.0;
  config.target.heightM = 0.0;
  config.target.toleranceMeters = 5'000.0;
  config.target.headingSearchHalfSpanDeg = 12.0;
  config.target.coarseHeadingCount = 17;
  config.target.coarseThetaCount = 21;
  config.target.refineIterations = 16;

  config.output.liveDelayMs = 18.0;
  config.output.liveIntegrationStepStride = 32;

  return config;
}

}  // namespace ballistics
