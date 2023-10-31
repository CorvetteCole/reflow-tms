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

void Logger::log(LogLevel severity, const char *message) const {
  if (severity >= logLevel) {
    StaticJsonDocument<256> logJson;
    logJson["time"] = millis();
    logJson["severity"] = logLevelStrings[severity];
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}

void Logger::log(LogLevel severity, const __FlashStringHelper *message) const {
  if (severity >= logLevel) {
    StaticJsonDocument<256> logJson;
    logJson["time"] = millis();
    logJson["severity"] = logLevelStrings[severity];
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}
