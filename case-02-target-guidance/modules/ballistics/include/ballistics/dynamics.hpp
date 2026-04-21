#pragma once

#include "ballistics/types.hpp"

namespace ballistics {

double toRadians(double degrees);
double toDegrees(double radians);
double clamp(double value, double lo, double hi);
double wrapLongitude(double longitudeRad);
double greatCircleDistance(double lambda0, double phi0, double lambda1,
                           double phi1);

State makeInitialState(double thetaDeg, double psiDeg);
void setCurrentControlProfile(const ControlProfile& profile);
void clearCurrentControlProfile();
double evaluateAlphaDeg(const ControlProfile& profile, double height);
double evaluateBetaDeg(const ControlProfile& profile, double height);
Atmosphere computeAtmosphere(double height);
double computeDragCoefficient(double mach, double alphaRad);
double computeLiftOrSideCoefficient(double mach, double angleRad);
Evaluation evaluateState(const State& state, double alphaRad,
                         double betaRad);
Evaluation evaluateState(const State& state);

double componentAt(const State& state, int index);
State stateFromComponents(const double values[6]);

}  // namespace ballistics
