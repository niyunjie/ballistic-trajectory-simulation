#include "ballistics/logging.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <utility>

#define NOMINMAX
#include <windows.h>

namespace ballistics {

namespace {

std::mutex gLogMutex;
std::wstring gLogFilePath;
std::terminate_handler gPreviousTerminate = nullptr;

std::string timestampNow() {
  SYSTEMTIME st;
  GetLocalTime(&st);
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u:%02u.%03u",
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
                st.wMilliseconds);
  return buffer;
}

void appendLogLine(const std::string& level, const std::string& component,
                   const std::string& message) {
  std::lock_guard<std::mutex> lock(gLogMutex);
  if (gLogFilePath.empty()) {
    return;
  }

  std::error_code ec;
  const std::filesystem::path path(gLogFilePath);
  std::filesystem::create_directories(path.parent_path(), ec);

  FILE* file = _wfopen(gLogFilePath.c_str(), L"ab");
  if (file == nullptr) {
    return;
  }

  std::ostringstream line;
  line << "[" << timestampNow() << "]"
       << " [" << level << "]"
       << " [" << component << "] " << message << "\r\n";
  const std::string text = line.str();
  std::fwrite(text.data(), 1, text.size(), file);
  std::fclose(file);
}

LONG WINAPI UnhandledExceptionLogger(EXCEPTION_POINTERS* exceptionInfo) {
  std::ostringstream message;
  if (exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr) {
    message << "Unhandled SEH exception code=0x" << std::hex
            << exceptionInfo->ExceptionRecord->ExceptionCode << std::dec
            << ", address=" << exceptionInfo->ExceptionRecord->ExceptionAddress;
  } else {
    message << "Unhandled SEH exception with no exception record.";
  }
  appendLogLine("FATAL", "crash", message.str());
  return EXCEPTION_CONTINUE_SEARCH;
}

void TerminateLogger() {
  try {
    const auto current = std::current_exception();
    if (current) {
      std::rethrow_exception(current);
    }
    appendLogLine("FATAL", "terminate", "std::terminate called without active exception.");
  } catch (const std::exception& ex) {
    appendLogLine("FATAL", "terminate", std::string("std::terminate: ") + ex.what());
  } catch (...) {
    appendLogLine("FATAL", "terminate", "std::terminate: unknown exception.");
  }

  if (gPreviousTerminate != nullptr) {
    gPreviousTerminate();
  }
  std::abort();
}

}  // namespace

void initializeLogging(const std::wstring& filePath) {
  {
    std::lock_guard<std::mutex> lock(gLogMutex);
    gLogFilePath = filePath;
  }
  appendLogLine("INFO", "logging", "Logging initialized.");
}

void installCrashHandlers() {
  SetUnhandledExceptionFilter(UnhandledExceptionLogger);
  gPreviousTerminate = std::set_terminate(TerminateLogger);
  appendLogLine("INFO", "logging", "Crash handlers installed.");
}

void logInfo(const std::string& component, const std::string& message) {
  appendLogLine("INFO", component, message);
}

void logError(const std::string& component, const std::string& message) {
  appendLogLine("ERROR", component, message);
}

void logException(const std::string& component, const std::exception& ex) {
  appendLogLine("ERROR", component, ex.what());
}

}  // namespace ballistics
