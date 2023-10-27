//
// Created by coleg on 10/27/23.
//

#include "logger.h"
void Logger::debug(const char *message) { log(LogLevel::DEBUG, message); }
void Logger::info(const char *message) { log(LogLevel::INFO, message); }
void Logger::warn(const char *message) { log(LogLevel::WARN, message); }
void Logger::error(const char *message) { log(LogLevel::ERROR, message); }
void Logger::log(LogLevel severity, const char *message) {
  if (severity >= logLevel) {
    // build json object, use length of message and log level

    // TODO use length of message and log level to make sure we don't overflow
    const char *severityString = logLevelToString(severity);
    strlen(severityString) + strlen(message);

    DynamicJsonDocument logJson(1024);
    logJson["action"] = "log";
    logJson["time"] = millis();
    logJson["severity"] = severityString;
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
  case ERROR:
    return "ERROR";
  }
  return "UNKNOWN";
}
