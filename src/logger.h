#ifndef REFLOW_TMS_LOGGER_H
#define REFLOW_TMS_LOGGER_H

#include <Arduino.h>
#include <ArduinoJson.h>

enum LogLevel { DEBUG = 0, INFO = 1, WARN = 2, CRITICAL = 3 };

class Logger {

public:
  LogLevel logLevel = LogLevel::INFO;

  void debug(const String &message) const;

  void debug(const __FlashStringHelper *message) const;

  void info(const String &message) const;

  void info(const __FlashStringHelper *message) const;

  void warn(const String &message) const;

  void warn(const __FlashStringHelper *message) const;

  void error(const String &message) const;

  void error(const __FlashStringHelper *message) const;

  void log(LogLevel severity, const String &message) const;

  void log(LogLevel severity, const __FlashStringHelper *message) const;

  explicit Logger(LogLevel logLevel) : logLevel(logLevel) {}

  Logger() = default;

private:
  const char *const logLevelStrings[4] = {"DEBUG", "INFO", "WARN", "CRITICAL"};
};

#endif // REFLOW_TMS_LOGGER_H
