//
// Created by coleg on 10/27/23.
//

#include "logger.h"
void Logger::debug(const char *message) const { log(LogLevel::DEBUG, message); }
void Logger::debug(const __FlashStringHelper *message) const {
  log(LogLevel::DEBUG, message);
}
void Logger::info(const char *message) const { log(LogLevel::INFO, message); }
void Logger::info(const __FlashStringHelper *message) const {
  log(LogLevel::INFO, message);
}
void Logger::warn(const char *message) const { log(LogLevel::WARN, message); }
void Logger::warn(const __FlashStringHelper *message) const {
  log(LogLevel::WARN, message);
}
void Logger::error(const char *message) const {
  log(LogLevel::CRITICAL, message);
}
void Logger::error(const __FlashStringHelper *message) const {
  log(LogLevel::CRITICAL, message);
}
void Logger::log(LogLevel severity, const String& message) const {
  if (severity >= logLevel) {
    DynamicJsonDocument logJson(256);
    logJson["action"] = "log";
    logJson["time"] = millis();
    logJson["severity"] = logLevelToString(severity);
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}

void Logger::log(LogLevel severity, const __FlashStringHelper *message) const {
  if (severity >= logLevel) {
    DynamicJsonDocument logJson(256);
    logJson["action"] = "log";
    logJson["time"] = millis();
    logJson["severity"] = logLevelToString(severity);
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}

char const *Logger::logLevelToString(LogLevel severity) {
  switch (severity) {
  case DEBUG:
    return "DEBUG";
  case INFO:
    return "INFO";
  case WARN:
    return "WARN";
  case CRITICAL:
    return "CRITICAL";
  }
  return "UNKNOWN";
}
