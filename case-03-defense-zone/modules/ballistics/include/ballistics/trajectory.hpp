#pragma once

#include <functional>

#include "ballistics/types.hpp"

namespace ballistics {

using StepCallback = std::function<void(const std::vector<Sample>&)>;

TrajectoryResult simulateTrajectory(double thetaDeg, double psiDeg,
                                    const IntegrationConfig& config,
                                    const StepCallback& liveStepCallback,
                                    bool enableLiveCallback);

}  // namespace ballistics
