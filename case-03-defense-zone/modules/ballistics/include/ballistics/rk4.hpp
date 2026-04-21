#pragma once

#include "ballistics/types.hpp"

namespace ballistics {

State rk4Step(const State& state, double dt);
double stateErrorNorm(const State& a, const State& b);

}  // namespace ballistics
