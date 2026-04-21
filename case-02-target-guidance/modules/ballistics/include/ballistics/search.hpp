#pragma once

#include "ballistics/live_writer.hpp"
#include "ballistics/solver_control.hpp"
#include "ballistics/types.hpp"

namespace ballistics {

FinalResult computeAll(LiveWriter& liveWriter,
                       const SolverControl* control = nullptr);

}  // namespace ballistics
