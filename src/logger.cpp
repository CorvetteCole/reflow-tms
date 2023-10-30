#include "logger.h"

const char *const LogLevelStrings[] = {"DEBUG", "INFO", "WARN", "CRITICAL"};

void Logger::debug(const String &message) const {
  log(LogLevel::DEBUG, message);
}
void Logger::debug(const __FlashStringHelper *message) const {
  log(LogLevel::DEBUG, message);
}
void Logger::info(const String &message) const { log(LogLevel::INFO, message); }
void Logger::info(const __FlashStringHelper *message) const {
  log(LogLevel::INFO, message);
}
void Logger::warn(const String &message) const { log(LogLevel::WARN, message); }
void Logger::warn(const __FlashStringHelper *message) const {
  log(LogLevel::WARN, message);
}
void Logger::error(const String &message) const {
  log(LogLevel::CRITICAL, message);
}
void Logger::error(const __FlashStringHelper *message) const {
  log(LogLevel::CRITICAL, message);
}

void Logger::log(LogLevel severity, const String &message) const {
  if (severity >= logLevel) {
    StaticJsonDocument<256> logJson;
    logJson["time"] = millis();
    logJson["severity"] = LogLevelStrings[severity];
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}

void Logger::log(LogLevel severity, const __FlashStringHelper *message) const {
  if (severity >= logLevel) {
    StaticJsonDocument<256> logJson;
    logJson["time"] = millis();
    logJson["severity"] = LogLevelStrings[severity];
    logJson["message"] = message;

    serializeJson(logJson, Serial);
    Serial.println();
  }
}
