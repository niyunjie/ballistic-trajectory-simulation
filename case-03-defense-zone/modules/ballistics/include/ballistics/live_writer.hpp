#pragma once

#include <functional>
#include <string>
#include <vector>

#include "ballistics/types.hpp"

namespace ballistics {

class LiveWriter {
 public:
  using Callback = std::function<void(const LiveState&)>;

  LiveWriter();
  explicit LiveWriter(std::wstring outputPath);
  explicit LiveWriter(Callback callback);
  LiveWriter(std::wstring outputPath, Callback callback);

  void writeRunningState(const std::string& phase, const std::string& message,
                         double progress, const CandidateSummary& current,
                         const std::vector<PlotPoint>& currentTrajectory,
                         const CandidateSummary& best,
                         const std::vector<PlotPoint>& bestTrajectory,
                         const std::vector<BoundaryPoint>& boundary,
                         const std::vector<BoundaryPoint>& effectiveBoundary,
                         const std::vector<std::vector<PlotPoint>>& sampleTrajs,
                         const std::vector<double>& sampleHeadings);

  void writeFinalState(const FinalResult& finalResult);

 private:
  void publish(const LiveState& state);
  void printConsoleProgress(const std::string& phase, const std::string& message,
                            double progress);
  void atomicWrite(const std::string& jsonText);

  std::wstring outputPath_;
  Callback callback_;
  std::string lastConsolePhase_;
  int lastConsolePercent_ = -1;
  std::string lastLoggedPhase_;
  int lastLoggedProgressBucket_ = -1;
};

std::string formatDouble(double value, int precision = 6);

}  // namespace ballistics
