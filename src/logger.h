//
// Created by coleg on 10/27/23.
//

#ifndef REFLOW_TMS_LOGGER_H
#define REFLOW_TMS_LOGGER_H

#include <Arduino.h>
#include <ArduinoJson.h>

enum LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

class Logger {

public:
  void debug(const char *message);

  void info(const char *message);

  void warn(const char *message);

  void error(const char *message);

  void log(LogLevel severity, const char *message);

  explicit Logger(LogLevel logLevel) : logLevel(logLevel) {}

  Logger() = default;

private:
  static char const *logLevelToString(LogLevel severity);

  LogLevel logLevel = INFO;
};

#endif // REFLOW_TMS_LOGGER_H
