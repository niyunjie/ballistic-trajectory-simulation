#include "ballistics/config.hpp"

namespace ballistics {

namespace {

SolverConfig gConfig;

}  // namespace

void setCurrentConfig(const SolverConfig& config) {
  gConfig = config;
}

const SolverConfig& currentConfig() {
  return gConfig;
}

}  // namespace ballistics
