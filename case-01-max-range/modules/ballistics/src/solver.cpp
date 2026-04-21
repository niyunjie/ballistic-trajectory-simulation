#include "ballistics/solver.hpp"

#include <sstream>

#include "ballistics/config.hpp"
#include "ballistics/logging.hpp"
#include "ballistics/search.hpp"

namespace ballistics {

FinalResult runSolver(const SolverConfig& config, LiveWriter* liveWriter) {
  setCurrentConfig(config);

  std::ostringstream startLine;
  startLine << "runSolver start: launch=(" << config.vehicle.initialLongitudeDeg << ", "
            << config.vehicle.initialLatitudeDeg << ", "
            << config.vehicle.initialHeight << "m), targetEnabled="
            << (config.target.enabled ? "true" : "false")
            << ", methods=[fixedLinear=" << (config.methods.fixedLinear ? "on" : "off")
            << ", fixedHermite=" << (config.methods.fixedHermite ? "on" : "off")
            << ", adaptiveHermite="
            << (config.methods.adaptiveHermite ? "on" : "off")
            << ", geneticAdaptive="
            << (config.methods.geneticAdaptive ? "on" : "off") << "]";
  logInfo("solver", startLine.str());

  LiveWriter nullWriter;
  LiveWriter& writer = liveWriter != nullptr ? *liveWriter : nullWriter;
  FinalResult result = computeAll(writer);

  std::ostringstream doneLine;
  doneLine << "runSolver complete: optimalThetaDeg=" << result.optimalThetaDeg
           << ", optimalRangeM=" << result.optimalRange
           << ", comparisons=" << result.comparisons.size();
  logInfo("solver", doneLine.str());
  return result;
}

}  // namespace ballistics
