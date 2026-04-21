#pragma once

#include "ballistics/config.hpp"
#include "ballistics/live_writer.hpp"
#include "ballistics/types.hpp"

namespace ballistics {

FinalResult runSolver(const SolverConfig& config, LiveWriter* liveWriter = nullptr);

}  // namespace ballistics
