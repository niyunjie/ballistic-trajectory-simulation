#include "ballistics/live_writer.hpp"

#include <algorithm>
#include <cstdio>
#include <cwchar>
#include <iomanip>
#include <iostream>
#include <sstream>

#define NOMINMAX
#include <windows.h>

#include "ballistics/config.hpp"
#include "ballistics/constants.hpp"
#include "ballistics/dynamics.hpp"
#include "ballistics/logging.hpp"

namespace ballistics {

namespace {

std::string formatPlotPoints(const std::vector<PlotPoint>& points) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "{"
        << "\"t\":" << formatDouble(points[i].t, 4) << ","
        << "\"lonDeg\":" << formatDouble(points[i].longitudeDeg, 6) << ","
        << "\"latDeg\":" << formatDouble(points[i].latitudeDeg, 6) << ","
        << "\"heightM\":" << formatDouble(points[i].height, 3) << ","
        << "\"speed\":" << formatDouble(points[i].speed, 3) << ","
        << "\"thetaDeg\":" << formatDouble(points[i].thetaDeg, 4) << ","
        << "\"psiDeg\":" << formatDouble(points[i].psiDeg, 4) << "}";
  }
  out << "]";
  return out.str();
}

std::string formatBoundaryPoints(const std::vector<BoundaryPoint>& boundary) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < boundary.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "{"
        << "\"headingDeg\":" << formatDouble(boundary[i].headingDeg, 4) << ","
        << "\"thetaDeg\":" << formatDouble(boundary[i].thetaDeg, 4) << ","
        << "\"rangeM\":" << formatDouble(boundary[i].range, 2) << ","
        << "\"lonDeg\":" << formatDouble(boundary[i].longitudeDeg, 6) << ","
        << "\"latDeg\":" << formatDouble(boundary[i].latitudeDeg, 6) << ","
        << "\"topHeightM\":" << formatDouble(boundary[i].topHeight, 2) << "}";
  }
  out << "]";
  return out.str();
}

std::string formatTrajectoryCollection(
    const std::vector<std::vector<PlotPoint>>& trajectories,
    const std::vector<double>& headings) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < trajectories.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "{"
        << "\"headingDeg\":" << formatDouble(headings[i], 4) << ","
        << "\"points\":" << formatPlotPoints(trajectories[i]) << "}";
  }
  out << "]";
  return out.str();
}

std::string formatComparisons(const std::vector<ComparisonRecord>& comparisons) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < comparisons.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << "{"
        << "\"name\":\"" << comparisons[i].name << "\","
        << "\"thetaDeg\":" << formatDouble(comparisons[i].thetaDeg, 4) << ","
        << "\"psiDeg\":" << formatDouble(comparisons[i].psiDeg, 4) << ","
        << "\"rangeM\":" << formatDouble(comparisons[i].range, 2) << ","
        << "\"flightTimeS\":" << formatDouble(comparisons[i].flightTime, 3)
        << ","
        << "\"steps\":" << comparisons[i].steps << ","
        << "\"impactLonDeg\":"
        << formatDouble(comparisons[i].impactLongitudeDeg, 6) << ","
        << "\"impactLatDeg\":"
        << formatDouble(comparisons[i].impactLatitudeDeg, 6) << ","
        << "\"speedCurve\":" << formatPlotPoints(comparisons[i].speedCurve)
        << "}";
  }
  out << "]";
  return out.str();
}

std::string formatTargetSolution(const TargetSolution& solution) {
  std::ostringstream out;
  out << "{"
      << "\"enabled\":" << (solution.enabled ? "true" : "false") << ","
      << "\"reachable\":" << (solution.reachable ? "true" : "false") << ","
      << "\"targetLonDeg\":" << formatDouble(solution.targetLongitudeDeg, 6)
      << ","
      << "\"targetLatDeg\":" << formatDouble(solution.targetLatitudeDeg, 6)
      << ","
      << "\"targetHeightM\":" << formatDouble(solution.targetHeightM, 2) << ","
      << "\"targetRangeM\":" << formatDouble(solution.targetRange, 2) << ","
      << "\"missDistanceM\":" << formatDouble(solution.missDistance, 2) << ","
      << "\"impactDistanceM\":" << formatDouble(solution.impactDistance, 2) << ","
      << "\"thetaDeg\":" << formatDouble(solution.thetaDeg, 4) << ","
      << "\"psiDeg\":" << formatDouble(solution.psiDeg, 4) << ","
      << "\"closestLonDeg\":" << formatDouble(solution.closestLongitudeDeg, 6)
      << ","
      << "\"closestLatDeg\":" << formatDouble(solution.closestLatitudeDeg, 6)
      << ","
      << "\"closestHeightM\":" << formatDouble(solution.closestHeightM, 2)
      << ","
      << "\"closestTimeS\":" << formatDouble(solution.closestTime, 3) << ","
      << "\"flightTimeS\":" << formatDouble(solution.flightTime, 3) << ","
      << "\"peakAltitudeM\":" << formatDouble(solution.peakAltitude, 2) << ","
      << "\"trajectoryHermite\":" << formatPlotPoints(solution.trajectoryHermite)
      << ","
      << "\"trajectoryLinear\":" << formatPlotPoints(solution.trajectoryLinear)
      << "}";
  return out.str();
}

CandidateSummary summarizeFinal(const FinalResult& finalResult) {
  CandidateSummary summary;
  summary.thetaDeg = finalResult.optimalThetaDeg;
  summary.psiDeg = finalResult.optimalPsiDeg;
  summary.range = finalResult.optimalRange;
  summary.flightTime = finalResult.optimalTrajectory.impactTime;
  summary.maxAltitude = finalResult.optimalTrajectory.maxAltitude;
  summary.impactLongitudeDeg = finalResult.optimalTrajectory.impacted
                                   ? toDegrees(finalResult.optimalTrajectory.impactState.lambda)
                                   : 0.0;
  summary.impactLatitudeDeg = finalResult.optimalTrajectory.impacted
                                  ? toDegrees(finalResult.optimalTrajectory.impactState.phi)
                                  : 0.0;
  return summary;
}

}  // namespace

LiveWriter::LiveWriter() = default;

LiveWriter::LiveWriter(std::wstring outputPath)
    : outputPath_(std::move(outputPath)) {}

LiveWriter::LiveWriter(Callback callback) : callback_(std::move(callback)) {}

LiveWriter::LiveWriter(std::wstring outputPath, Callback callback)
    : outputPath_(std::move(outputPath)), callback_(std::move(callback)) {}

std::string formatDouble(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

void LiveWriter::printConsoleProgress(const std::string& phase,
                                      const std::string& message,
                                      double progress) {
  const int percent =
      static_cast<int>(std::clamp(progress, 0.0, 1.0) * 100.0 + 0.5);
  if (phase == lastConsolePhase_ && percent == lastConsolePercent_) {
    return;
  }

  constexpr int kBarWidth = 28;
  const int filled = percent * kBarWidth / 100;
  std::ostringstream line;
  line << "\r[";
  for (int i = 0; i < kBarWidth; ++i) {
    line << (i < filled ? '#' : '-');
  }
  line << "] " << std::setw(3) << percent << "% ";

  if (phase == "search_optimal") {
    if (message.find("global farthest trajectory") != std::string::npos) {
      line << "Searching farthest heading";
    } else {
      line << "Searching best theta";
    }
  } else if (phase == "attack_zone") {
    line << "Building attack zone";
  } else if (phase == "target_solution") {
    line << "Matching target";
  } else {
    line << phase;
  }

  line << " | " << message << "    ";
  std::cout << line.str() << std::flush;

  const int progressBucket = percent / 25;
  if (phase != lastLoggedPhase_ || progressBucket != lastLoggedProgressBucket_ ||
      percent == 100) {
    std::ostringstream logLine;
    logLine << phase << " " << percent << "% - " << message;
    logInfo("progress", logLine.str());
    lastLoggedPhase_ = phase;
    lastLoggedProgressBucket_ = progressBucket;
  }

  lastConsolePhase_ = phase;
  lastConsolePercent_ = percent;
}

void LiveWriter::writeRunningState(
    const std::string& phase, const std::string& message, double progress,
    const CandidateSummary& current,
    const std::vector<PlotPoint>& currentTrajectory, const CandidateSummary& best,
    const std::vector<PlotPoint>& bestTrajectory,
    const std::vector<BoundaryPoint>& boundary,
    const std::vector<std::vector<PlotPoint>>& sampleTrajs,
    const std::vector<double>& sampleHeadings) {
  LiveState state;
  state.status = SolverStatus::Running;
  state.phase = phase;
  state.message = message;
  state.progress = progress;
  state.current = current;
  state.best = best;
  state.currentTrajectory = currentTrajectory;
  state.bestTrajectory = bestTrajectory;
  state.attackBoundary = boundary;
  state.attackTrajectories = sampleTrajs;
  state.attackTrajectoryHeadings = sampleHeadings;

  std::ostringstream out;
  out << "{"
      << "\"status\":\"running\","
      << "\"phase\":\"" << phase << "\","
      << "\"message\":\"" << message << "\","
      << "\"progress\":" << formatDouble(progress, 4) << ","
      << "\"current\":{"
      << "\"thetaDeg\":" << formatDouble(current.thetaDeg, 4) << ","
      << "\"psiDeg\":" << formatDouble(current.psiDeg, 4) << ","
      << "\"rangeM\":" << formatDouble(current.range, 2) << ","
      << "\"flightTimeS\":" << formatDouble(current.flightTime, 3) << ","
      << "\"maxAltitudeM\":" << formatDouble(current.maxAltitude, 2) << ","
      << "\"impactLonDeg\":" << formatDouble(current.impactLongitudeDeg, 6)
      << ","
      << "\"impactLatDeg\":" << formatDouble(current.impactLatitudeDeg, 6)
      << "},"
      << "\"best\":{"
      << "\"thetaDeg\":" << formatDouble(best.thetaDeg, 4) << ","
      << "\"psiDeg\":" << formatDouble(best.psiDeg, 4) << ","
      << "\"rangeM\":" << formatDouble(best.range, 2) << ","
      << "\"flightTimeS\":" << formatDouble(best.flightTime, 3) << ","
      << "\"maxAltitudeM\":" << formatDouble(best.maxAltitude, 2) << ","
      << "\"impactLonDeg\":" << formatDouble(best.impactLongitudeDeg, 6) << ","
      << "\"impactLatDeg\":" << formatDouble(best.impactLatitudeDeg, 6)
      << "},"
      << "\"currentTrajectory\":" << formatPlotPoints(currentTrajectory) << ","
      << "\"bestTrajectory\":" << formatPlotPoints(bestTrajectory) << ","
      << "\"attackBoundary\":" << formatBoundaryPoints(boundary) << ","
      << "\"attackTrajectories\":"
      << formatTrajectoryCollection(sampleTrajs, sampleHeadings) << "}";

  publish(state);
  if (!outputPath_.empty()) {
    atomicWrite(out.str());
  }
  printConsoleProgress(phase, message, progress);

  const double delayMs = std::max(0.0, currentConfig().output.liveDelayMs);
  if (delayMs > 0.0) {
    Sleep(static_cast<DWORD>(delayMs));
  }
}

void LiveWriter::writeFinalState(const FinalResult& finalResult) {
  const auto& runtime = currentConfig();
  const PlotPoint fallbackImpact{
      finalResult.optimalTrajectory.impactTime,
      runtime.vehicle.initialLongitudeDeg,
      runtime.vehicle.initialLatitudeDeg,
      std::max(0.0, runtime.vehicle.initialHeight),
      finalResult.optimalTrajectory.impacted ? finalResult.optimalTrajectory.impactState.V
                                             : runtime.vehicle.initialSpeed,
      finalResult.optimalThetaDeg,
      runtime.vehicle.initialPsiDeg};
  const PlotPoint& impactPoint = finalResult.optimalHermitePoints.empty()
                                     ? fallbackImpact
                                     : finalResult.optimalHermitePoints.back();

  LiveState state;
  state.status = SolverStatus::Done;
  state.phase = "complete";
  state.message = "Computation finished";
  state.progress = 1.0;
  state.current = summarizeFinal(finalResult);
  state.best = state.current;
  state.currentTrajectory = finalResult.optimalLinearPoints;
  state.bestTrajectory = finalResult.optimalHermitePoints;
  state.attackBoundary = finalResult.attackBoundary;
  state.attackTrajectories = finalResult.attackTrajectories;
  state.attackTrajectoryHeadings = finalResult.attackTrajectoryHeadings;
  state.hasFinalResult = true;
  state.finalResult = finalResult;

  std::ostringstream out;
  out << "{"
      << "\"status\":\"done\","
      << "\"phase\":\"complete\","
      << "\"message\":\"Computation finished\","
      << "\"summary\":{"
      << "\"launchLonDeg\":"
      << formatDouble(runtime.vehicle.initialLongitudeDeg, 4) << ","
      << "\"launchLatDeg\":"
      << formatDouble(runtime.vehicle.initialLatitudeDeg, 4) << ","
      << "\"launchHeightM\":" << formatDouble(runtime.vehicle.initialHeight, 2)
      << ","
      << "\"v0\":" << formatDouble(runtime.vehicle.initialSpeed, 2) << ","
      << "\"optimalThetaDeg\":" << formatDouble(finalResult.optimalThetaDeg, 4)
      << ","
      << "\"optimalPsiDeg\":" << formatDouble(finalResult.optimalPsiDeg, 4) << ","
      << "\"optimalRangeM\":" << formatDouble(finalResult.optimalRange, 2)
      << ","
      << "\"flightTimeS\":"
      << formatDouble(finalResult.optimalTrajectory.impactTime, 3) << ","
      << "\"peakAltitudeM\":"
      << formatDouble(finalResult.optimalTrajectory.maxAltitude, 2) << ","
      << "\"impactLonDeg\":"
      << formatDouble(impactPoint.longitudeDeg, 6) << ","
      << "\"impactLatDeg\":"
      << formatDouble(impactPoint.latitudeDeg, 6) << "},"
      << "\"comparisons\":" << formatComparisons(finalResult.comparisons) << ","
      << "\"optimalTrajectoryHermite\":"
      << formatPlotPoints(finalResult.optimalHermitePoints) << ","
      << "\"optimalTrajectoryLinear\":"
      << formatPlotPoints(finalResult.optimalLinearPoints) << ","
      << "\"speedCurveHermite\":"
      << formatPlotPoints(finalResult.speedCurveHermite) << ","
      << "\"speedCurveLinear\":"
      << formatPlotPoints(finalResult.speedCurveLinear) << ","
      << "\"attackBoundary\":"
      << formatBoundaryPoints(finalResult.attackBoundary) << ","
      << "\"attackTrajectories\":"
      << formatTrajectoryCollection(finalResult.attackTrajectories,
                                    finalResult.attackTrajectoryHeadings)
      << ","
      << "\"targetSolution\":"
      << formatTargetSolution(finalResult.targetSolution) << "}";

  publish(state);
  if (!outputPath_.empty()) {
    atomicWrite(out.str());
  }
  std::ostringstream logLine;
  logLine << "Final state written. optimalThetaDeg="
          << formatDouble(finalResult.optimalThetaDeg, 4)
          << ", optimalPsiDeg=" << formatDouble(finalResult.optimalPsiDeg, 4)
          << ", optimalRangeM=" << formatDouble(finalResult.optimalRange, 2)
          << ", comparisons=" << finalResult.comparisons.size()
          << ", targetEnabled="
          << (finalResult.targetSolution.enabled ? "true" : "false")
          << ", targetReachable="
          << (finalResult.targetSolution.reachable ? "true" : "false");
  logInfo("progress", logLine.str());
  std::cout << "\r[############################] 100% Complete | Computation finished"
            << std::string(24, ' ') << "\n";
}

void LiveWriter::publish(const LiveState& state) {
  if (callback_) {
    callback_(state);
  }
}

void LiveWriter::atomicWrite(const std::string& jsonText) {
  if (outputPath_.empty()) {
    return;
  }

  const std::wstring tempPath = outputPath_ + L".tmp";
  FILE* file = _wfopen(tempPath.c_str(), L"wb");
  if (file == nullptr) {
    return;
  }
  std::fwrite(jsonText.data(), 1, jsonText.size(), file);
  std::fclose(file);
  if (!MoveFileExW(tempPath.c_str(), outputPath_.c_str(),
                   MOVEFILE_REPLACE_EXISTING | MOVEFILE_COPY_ALLOWED)) {
    _wremove(outputPath_.c_str());
    _wrename(tempPath.c_str(), outputPath_.c_str());
  }
}

}  // namespace ballistics
