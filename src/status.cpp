#include "status.h"

DynamicJsonDocument Status::toJson() const volatile {
  DynamicJsonDocument statusJson(192);
  statusJson["targetTemperature"] = targetTemperature;
  statusJson["currentTemperature"] = currentTemperature;
  statusJson["topHeatDutyCycle"] = topHeatDutyCycle;
  statusJson["bottomHeatDutyCycle"] = bottomHeatDutyCycle;
  statusJson["isDoorOpen"] = isDoorOpen;
  statusJson["error"] = error;
  // convert state enum value to string
  switch (state) {
  case State::IDLE:
    statusJson["state"] = "IDLE";
    break;
  case State::HEATING:
    statusJson["state"] = "HEATING";
    break;
  case State::COOLING:
    statusJson["state"] = "COOLING";
    break;
  case State::ERROR:
    statusJson["state"] = "ERROR";
    break;
  }
  return statusJson;
}
