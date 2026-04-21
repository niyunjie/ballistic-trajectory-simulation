#pragma once

#include <exception>
#include <string>

namespace ballistics {

void initializeLogging(const std::wstring& filePath);
void installCrashHandlers();

void logInfo(const std::string& component, const std::string& message);
void logError(const std::string& component, const std::string& message);
void logException(const std::string& component, const std::exception& ex);

}  // namespace ballistics
