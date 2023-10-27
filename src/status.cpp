#include "status.h"

DynamicJsonDocument Status::toJson() const volatile {
  DynamicJsonDocument statusJson(1024);
  statusJson["targetTemperature"] = targetTemperature;
  statusJson["currentTemperature"] = currentTemperature;
  statusJson["topHeatDutyCycle"] = topHeatDutyCycle;
  statusJson["bottomHeatDutyCycle"] = bottomHeatDutyCycle;
  statusJson["isDoorOpen"] = isDoorOpen;
  statusJson["error"] = error;
  // convert state enum value to string
  switch (state) {
  case State::idle:
    statusJson["state"] = "idle";
    break;
  case State::heating:
    statusJson["state"] = "heating";
    break;
  case State::cooling:
    statusJson["state"] = "cooling";
    break;
  case State::error:
    statusJson["state"] = "error";
    break;
  }
  statusJson["isDiagnosticMode"] = isDiagnosticMode;
  return statusJson;
}
