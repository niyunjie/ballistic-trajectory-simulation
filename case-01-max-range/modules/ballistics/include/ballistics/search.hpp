#pragma once

#include <vector>

#include "ballistics/live_writer.hpp"
#include "ballistics/trajectory.hpp"
#include "ballistics/types.hpp"

namespace ballistics {

CandidateSummary summarizeCandidate(double thetaDeg, const TrajectoryResult& traj);
SearchOutcome evaluateTheta(double thetaDeg, double psiDeg,
                            const IntegrationConfig& config, bool enableLive,
                            const StepCallback& callback);
SearchOutcome searchOptimalTheta(
    double psiDeg, const IntegrationConfig& config, LiveWriter& liveWriter,
    std::vector<BoundaryPoint>& boundary,
    std::vector<std::vector<PlotPoint>>& attackTrajectories,
    std::vector<double>& attackHeadings);
void buildAttackZone(const IntegrationConfig& config, LiveWriter& liveWriter,
                     const SearchOutcome& optimalSearch,
                     std::vector<BoundaryPoint>& boundary,
                     std::vector<std::vector<PlotPoint>>& attackTrajectories,
                     std::vector<double>& attackHeadings);
FinalResult computeAll(LiveWriter& liveWriter);

}  // namespace ballistics
