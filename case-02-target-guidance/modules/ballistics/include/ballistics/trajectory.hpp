#pragma once

#include <functional>

#include "ballistics/solver_control.hpp"
#include "ballistics/types.hpp"

namespace ballistics {

using StepCallback = std::function<void(const std::vector<Sample>&)>;

TrajectoryResult simulateTrajectory(const ControlProfile& control,
                                    const IntegrationConfig& config,
                                    const StepCallback& liveStepCallback,
                                    bool enableLiveCallback,
                                    const SolverControl* solverControl = nullptr);

}  // namespace ballistics
