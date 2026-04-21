#include <cstdio>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "ballistics/defaults.hpp"
#include "ballistics/live_writer.hpp"
#include "ballistics/logging.hpp"
#include "ballistics/solver.hpp"

namespace {

std::filesystem::path outputDirectory() {
  return std::filesystem::current_path() / "data" / "solver_output";
}

std::filesystem::path logFilePath() {
  return std::filesystem::current_path() / "data" / "logs" / "solver_cli.log";
}

ballistics::SolverConfig buildUserConfig() {
  ballistics::SolverConfig config = ballistics::makeDefaultConfig();
  const std::filesystem::path outputDir = outputDirectory();
  config.output.livePath = (outputDir / "live-state.json").wstring();
  config.output.finalPath = (outputDir / "final-state.json").wstring();
  return config;
}

}  // namespace

int main() {
  ballistics::initializeLogging(logFilePath().wstring());
  ballistics::installCrashHandlers();
  try {
    const ballistics::SolverConfig config = buildUserConfig();
    std::filesystem::create_directories(outputDirectory());
    std::filesystem::create_directories(logFilePath().parent_path());
    ballistics::logInfo("solver_cli",
                        std::string("CLI solve starting. finalPath=") +
                            std::filesystem::path(config.output.finalPath).string());

    ballistics::LiveWriter liveWriter(config.output.livePath);
    const ballistics::FinalResult finalResult =
        ballistics::runSolver(config, &liveWriter);
    liveWriter.writeFinalState(finalResult);

    const std::string summaryJson =
        std::string("{") + "\"optimalThetaDeg\":" +
        ballistics::formatDouble(finalResult.optimalThetaDeg, 4) + "," +
        "\"optimalPsiDeg\":" +
        ballistics::formatDouble(finalResult.optimalPsiDeg, 4) + "," +
        "\"optimalRangeM\":" +
        ballistics::formatDouble(finalResult.optimalRange, 2) + "}";
    FILE* summaryFile = _wfopen(config.output.finalPath.c_str(), L"wb");
    if (summaryFile != nullptr) {
      std::fwrite(summaryJson.data(), 1, summaryJson.size(), summaryFile);
      std::fclose(summaryFile);
    }

    std::cout << "Optimal theta (deg): " << std::fixed << std::setprecision(4)
              << finalResult.optimalThetaDeg << "\n";
    std::cout << "Optimal heading psi (deg): " << std::fixed
              << std::setprecision(4) << finalResult.optimalPsiDeg << "\n";
    std::cout << "Optimal range (m): " << std::fixed << std::setprecision(2)
              << finalResult.optimalRange << "\n";
    ballistics::logInfo("solver_cli", "CLI solve finished successfully.");
    return 0;
  } catch (const std::exception& ex) {
    ballistics::logException("solver_cli", ex);
    std::cerr << "Solver failed: " << ex.what() << "\n";
    return 1;
  } catch (...) {
    ballistics::logError("solver_cli", "Solver failed with unknown exception.");
    std::cerr << "Solver failed: unknown exception\n";
    return 1;
  }
}
