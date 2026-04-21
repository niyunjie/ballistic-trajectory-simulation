#pragma once

#include "ballistics/config.hpp"
#include "ballistics/live_writer.hpp"
#include "ballistics/solver_control.hpp"
#include "ballistics/types.hpp"

namespace ballistics {

FinalResult runSolver(const SolverConfig& config, LiveWriter* liveWriter = nullptr,
                      const SolverControl* control = nullptr);

}  // namespace ballistics
