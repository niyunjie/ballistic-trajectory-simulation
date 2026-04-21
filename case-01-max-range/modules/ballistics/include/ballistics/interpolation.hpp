#pragma once

#include <vector>

#include "ballistics/types.hpp"

namespace ballistics {

Sample makeSample(double time, const State& state);
State interpolateLinearState(const Sample& a, const Sample& b, double u);
State interpolateHermiteState(const Sample& a, const Sample& b, double u);
double interpolateImpactFraction(const Sample& a, const Sample& b,
                                 ImpactInterpolation interpolation);
State interpolateTrajectoryState(const std::vector<Sample>& samples, double time,
                                 ResampleInterpolation interpolation);
std::vector<PlotPoint> resampleTrajectory(const TrajectoryResult& trajectory,
                                          ResampleInterpolation interpolation,
                                          int pointCount);

}  // namespace ballistics
