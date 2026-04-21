#include "ballistics/dynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"

namespace ballistics {

namespace {

thread_local ControlProfile gCurrentControlProfile;
thread_local bool gHasCurrentControlProfile = false;

constexpr std::array<double, ControlProfile::kAlphaNodeCount + 1>
    kAlphaFractions = {1.0, 0.75, 0.5, 0.25, 0.0};
constexpr std::array<double, ControlProfile::kBetaNodeCount>
    kBetaFractions = {1.0, 0.75, 0.5, 0.25, 0.0};

double lerp(double a, double b, double t) {
  return a + (b - a) * t;
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

double headingDifferenceDeg(double lhs, double rhs) {
  double diff = wrapHeadingDeg(lhs - rhs);
  if (diff > 180.0) {
    diff -= 360.0;
  }
  return diff;
}

double smoothstep(double edge0, double edge1, double x) {
  const double width = edge1 - edge0;
  if (std::abs(width) < 1e-9) {
    return x >= edge1 ? 1.0 : 0.0;
  }
  const double t = clamp((x - edge0) / width, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double configuredAlphaRad() {
  return toRadians(clamp(currentConfig().vehicle.initialAlphaDeg, 0.0, 14.0));
}

double configuredBetaRad() {
  return toRadians(currentConfig().vehicle.initialBetaDeg);
}

template <std::size_t N>
double interpolateFractionProfile(const std::array<double, N>& values,
                                  const std::array<double, N>& fractions,
                                  double fraction) {
  const double clampedFraction = clamp(fraction, 0.0, 1.0);
  if (clampedFraction >= fractions.front()) {
    return values.front();
  }
  if (clampedFraction <= fractions.back()) {
    return values.back();
  }

  for (std::size_t i = 1; i < fractions.size(); ++i) {
    if (clampedFraction >= fractions[i]) {
      const double span = fractions[i - 1] - fractions[i];
      const double blend =
          span <= 1e-9 ? 0.0 : (fractions[i - 1] - clampedFraction) / span;
      return lerp(values[i - 1], values[i], blend);
    }
  }
  return values.back();
}

double optimizedAlphaDeg(const ControlProfile& profile, double height) {
  const auto& runtime = currentConfig();
  const double transitionHeight = clamp(30'000.0, runtime.search.controlHeightMinM,
                                        runtime.search.controlHeightMaxM);
  if (height >= transitionHeight) {
    return 0.0;
  }
  return clamp(0.85 * (profile.targetThetaDeg - runtime.vehicle.initialThetaDeg),
               runtime.search.alphaMinDeg, runtime.search.alphaMaxDeg);
}

double optimizedBetaDeg(const ControlProfile& profile, double height) {
  (void)height;
  const auto& runtime = currentConfig();
  return clamp(1.25 * headingDifferenceDeg(profile.targetPsiDeg,
                                           runtime.vehicle.initialPsiDeg),
               runtime.search.betaMinDeg, runtime.search.betaMaxDeg);
}

double scheduledAlphaRad(const State& state) {
  if (gHasCurrentControlProfile) {
    const auto& runtime = currentConfig();
    const double height = state.r - runtime.earth.radius;
    const double transitionHeight =
        clamp(30'000.0, runtime.search.controlHeightMinM,
              runtime.search.controlHeightMaxM);
    if (height >= transitionHeight) {
      return 0.0;
    }
    const double thetaErrorDeg =
        gCurrentControlProfile.targetThetaDeg - toDegrees(state.theta);
    return toRadians(clamp(0.85 * thetaErrorDeg, runtime.search.alphaMinDeg,
                           runtime.search.alphaMaxDeg));
  }

  const auto& vehicle = currentConfig().vehicle;
  const double height = state.r - currentConfig().earth.radius;
  const double peakAlpha = configuredAlphaRad();
  const double lowAltitudeAlpha = peakAlpha * 0.65;
  const double qianMidAlpha = peakAlpha * 1.35;
  const double aggressiveMidAlpha = peakAlpha * 1.60;
  const double thetaDeg = toDegrees(state.theta);

  switch (vehicle.alphaScheduleMode) {
    case AlphaScheduleMode::ConstantBelow30km:
      return height >= 30'000.0 ? 0.0 : peakAlpha;

    case AlphaScheduleMode::SmoothGlide:
      if (height >= 30'000.0) {
        return 0.0;
      }
      if (height >= 20'000.0) {
        const double blend = smoothstep(30'000.0, 20'000.0, height);
        return lerp(0.0, peakAlpha, blend);
      }
      if (height >= 10'000.0) {
        return peakAlpha;
      }
      return lerp(lowAltitudeAlpha, peakAlpha, smoothstep(10'000.0, 0.0, height));

    case AlphaScheduleMode::QianStyle:
      if (height >= 45'000.0) {
        return 0.0;
      }
      if (height >= 30'000.0) {
        return lerp(0.0, peakAlpha * 0.45, smoothstep(45'000.0, 30'000.0, height));
      }
      if (height >= 18'000.0) {
        return lerp(peakAlpha * 0.45, qianMidAlpha,
                    smoothstep(30'000.0, 18'000.0, height));
      }
      if (height >= 12'000.0) {
        return qianMidAlpha;
      }
      if (height >= 4'000.0) {
        return lerp(qianMidAlpha, peakAlpha * 0.75,
                    smoothstep(12'000.0, 4'000.0, height));
      }
      return peakAlpha * 0.75;

    case AlphaScheduleMode::AggressiveSkip:
      if (height >= 55'000.0) {
        return 0.0;
      }
      if (height >= 38'000.0) {
        return lerp(0.0, peakAlpha * 0.35, smoothstep(55'000.0, 38'000.0, height));
      }
      if (height >= 24'000.0) {
        return lerp(peakAlpha * 0.35, aggressiveMidAlpha,
                    smoothstep(38'000.0, 24'000.0, height));
      }
      if (height >= 16'000.0) {
        return aggressiveMidAlpha;
      }
      if (height >= 8'000.0) {
        return lerp(aggressiveMidAlpha, peakAlpha * 0.95,
                    smoothstep(16'000.0, 8'000.0, height));
      }
      return peakAlpha * 0.95;

    case AlphaScheduleMode::QianSkipGlide: {
      if (height >= 68'000.0) {
        return 0.0;
      }

      const double entryBlend = smoothstep(68'000.0, 50'000.0, height);
      const double diveBlend = smoothstep(-1.5, -10.0, thetaDeg);
      const double reboundBlend = smoothstep(1.0, 9.0, thetaDeg);
      const double secondDiveBlend = smoothstep(-0.5, -7.0, thetaDeg);
      const double recoveryBlend = smoothstep(24'000.0, 12'000.0, height);

      const double entryAlpha = lerp(0.0, peakAlpha * 0.24, entryBlend);
      const double pullUpAlpha =
          lerp(entryAlpha, peakAlpha * 3.00, entryBlend * diveBlend);
      const double releasedAlpha =
          lerp(pullUpAlpha, peakAlpha * 0.10, reboundBlend);

      if (height >= 26'000.0) {
        return releasedAlpha;
      }

      const double secondaryLift =
          lerp(releasedAlpha, peakAlpha * 1.35, recoveryBlend * secondDiveBlend);
      if (height >= 12'000.0) {
        return secondaryLift;
      }

      return peakAlpha * 0.55;
    }

    case AlphaScheduleMode::SangerStyle: {
      if (height >= 72'000.0) {
        return 0.0;
      }

      const double preEntryBlend = smoothstep(72'000.0, 56'000.0, height);
      const double upperGlideBlend = smoothstep(56'000.0, 38'000.0, height);
      const double shallowFlightBlend = smoothstep(-4.0, 2.0, thetaDeg);
      const double terminalBlend = smoothstep(24'000.0, 10'000.0, height);

      const double preEntryAlpha = lerp(0.0, peakAlpha * 0.28, preEntryBlend);
      const double upperGlideAlpha = lerp(preEntryAlpha, peakAlpha * 1.65,
                                          upperGlideBlend);
      const double cruiseAlpha =
          lerp(upperGlideAlpha, peakAlpha * 1.05, shallowFlightBlend);

      if (height >= 24'000.0) {
        return cruiseAlpha;
      }

      return lerp(cruiseAlpha, peakAlpha * 0.70, terminalBlend);
    }
  }

  return peakAlpha;
}

double scheduledBetaRad(const State& state) {
  if (gHasCurrentControlProfile) {
    const auto& runtime = currentConfig();
    const double psiErrorDeg =
        headingDifferenceDeg(gCurrentControlProfile.targetPsiDeg,
                             toDegrees(state.psi));
    return toRadians(clamp(1.25 * psiErrorDeg, runtime.search.betaMinDeg,
                           runtime.search.betaMaxDeg));
  }

  return configuredBetaRad();
}

}  // namespace

double toRadians(double degrees) {
  return degrees * kPi / 180.0;
}

double toDegrees(double radians) {
  return radians * 180.0 / kPi;
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

double wrapLongitude(double longitudeRad) {
  while (longitudeRad > kPi) {
    longitudeRad -= 2.0 * kPi;
  }
  while (longitudeRad < -kPi) {
    longitudeRad += 2.0 * kPi;
  }
  return longitudeRad;
}

double greatCircleDistance(double lambda0, double phi0, double lambda1,
                           double phi1) {
  const auto& config = currentConfig();
  const double centralAngle =
      std::acos(clamp(std::sin(phi0) * std::sin(phi1) +
                          std::cos(phi0) * std::cos(phi1) *
                              std::cos(lambda1 - lambda0),
                      -1.0, 1.0));
  return config.earth.radius * centralAngle;
}

State makeInitialState(double thetaDeg, double psiDeg) {
  const auto& config = currentConfig();
  return {config.vehicle.initialSpeed,
          toRadians(thetaDeg),
          toRadians(psiDeg),
          config.earth.radius + config.vehicle.initialHeight,
          toRadians(config.vehicle.initialLongitudeDeg),
          toRadians(config.vehicle.initialLatitudeDeg)};
}

void setCurrentControlProfile(const ControlProfile& profile) {
  gCurrentControlProfile = profile;
  gHasCurrentControlProfile = true;
}

void clearCurrentControlProfile() {
  gHasCurrentControlProfile = false;
}

double evaluateAlphaDeg(const ControlProfile& profile, double height) {
  return optimizedAlphaDeg(profile, height);
}

double evaluateBetaDeg(const ControlProfile& profile, double height) {
  return optimizedBetaDeg(profile, height);
}

Atmosphere computeAtmosphere(double height) {
  const double cappedHeight = std::max(0.0, height);
  const double temperature =
      std::max(200.0, 288.15 - 0.0065 * cappedHeight);
  const double density = 1.225 * std::exp(-0.00015 * cappedHeight);
  const double soundSpeed = 20.0468 * std::sqrt(temperature);
  return {density, temperature, soundSpeed, 0.0};
}

double computeDragCoefficient(double mach, double alphaRad) {
  const double ma[3] = {mach * mach, mach, 1.0};
  const double alpha[3] = {alphaRad * alphaRad, alphaRad, 1.0};
  const double matrix[3][3] = {
      {-0.0002, 0.0038, 0.1582},
      {-0.0022, -0.0132, -0.8520},
      {0.0115, -0.0044, 1.9712},
  };
  double total = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      total += ma[i] * matrix[i][j] * alpha[j];
    }
  }
  return total;
}

double computeLiftOrSideCoefficient(double mach, double angleRad) {
  const std::array<double, 3> weights = {-0.026, 0.0651, 0.4913};
  const std::array<double, 3> ma = {mach * mach, mach, 1.0};
  double base = 0.0;
  for (std::size_t i = 0; i < weights.size(); ++i) {
    base += ma[i] * weights[i];
  }
  return base * angleRad;
}

Evaluation evaluateState(const State& state, double alphaRad, double betaRad) {
  const auto& config = currentConfig();
  const double height = state.r - config.earth.radius;
  Atmosphere atmosphere = computeAtmosphere(height);
  atmosphere.mach =
      atmosphere.soundSpeed > 1e-9 ? state.V / atmosphere.soundSpeed : 0.0;

  const double cd = computeDragCoefficient(atmosphere.mach, alphaRad);
  const double cl = computeLiftOrSideCoefficient(atmosphere.mach, alphaRad);
  const double cy = computeLiftOrSideCoefficient(atmosphere.mach, betaRad);
  const double dynamicPressure =
      0.5 * atmosphere.density * state.V * state.V * config.vehicle.refArea;
  const double drag = dynamicPressure * cd;
  const double lift = dynamicPressure * cl;
  const double sideForce = dynamicPressure * cy;
  const double gravity = config.earth.mu / (state.r * state.r);

  const double sinTheta = std::sin(state.theta);
  const double cosTheta = std::cos(state.theta);
  const double sinPsi = std::sin(state.psi);
  const double cosPsi = std::cos(state.psi);
  const double sinPhi = std::sin(state.phi);
  const double cosPhi = std::cos(state.phi);

  const double safeCosTheta =
      std::abs(cosTheta) < 1e-8 ? (cosTheta >= 0.0 ? 1e-8 : -1e-8) : cosTheta;
  const double safeCosPhi =
      std::abs(cosPhi) < 1e-8 ? (cosPhi >= 0.0 ? 1e-8 : -1e-8) : cosPhi;
  const double omega2r = state.r * config.earth.omega * config.earth.omega;

  const double vDot =
      -drag / config.vehicle.mass - gravity * sinTheta +
      omega2r * cosPhi * (sinTheta * cosPhi - cosTheta * sinPhi * cosPsi);
  const double thetaDot =
      lift / (config.vehicle.mass * std::max(state.V, 1.0)) -
      gravity * cosTheta / std::max(state.V, 1.0) +
      state.V * cosTheta / state.r +
      2.0 * config.earth.omega * sinPsi * cosPhi +
      (omega2r / std::max(state.V, 1.0)) * cosPhi *
          (cosTheta * cosPhi + sinTheta * sinPhi * cosPsi);
  const double psiDot =
      sideForce /
          (config.vehicle.mass * std::max(state.V, 1.0) * safeCosTheta) +
      (omega2r * sinPhi * cosPhi * sinPsi) /
          (std::max(state.V, 1.0) * safeCosTheta) +
      state.V * cosTheta * sinPsi * std::tan(state.phi) / state.r -
      2.0 * config.earth.omega *
          (std::tan(state.theta) * cosPsi * cosPhi - sinPhi);
  const double rDot = state.V * sinTheta;
  const double lambdaDot = state.V * cosTheta * sinPsi / (state.r * safeCosPhi);
  const double phiDot = state.V * cosTheta * cosPsi / state.r;

  return {{vDot, thetaDot, psiDot, rDot, lambdaDot, phiDot},
          atmosphere,
          {cd, cl, cy, drag, lift, sideForce, gravity}};
}

Evaluation evaluateState(const State& state) {
  return evaluateState(state, scheduledAlphaRad(state), scheduledBetaRad(state));
}

double componentAt(const State& state, int index) {
  switch (index) {
    case 0:
      return state.V;
    case 1:
      return state.theta;
    case 2:
      return state.psi;
    case 3:
      return state.r;
    case 4:
      return state.lambda;
    default:
      return state.phi;
  }
}

State stateFromComponents(const double values[6]) {
  return {values[0], values[1], values[2], values[3], values[4], values[5]};
}

}  // namespace ballistics
