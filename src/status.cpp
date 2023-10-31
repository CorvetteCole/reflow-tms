#include "status.h"

StaticJsonDocument<96> Status::toJson() const {
  StaticJsonDocument<96> statusJson;
  statusJson["targetTemperature"] = targetTemperature;
  statusJson["currentTemperature"] = currentTemperature;
  statusJson["topHeatDutyCycle"] = topHeatDutyCycle;
  statusJson["bottomHeatDutyCycle"] = bottomHeatDutyCycle;
  statusJson["isDoorOpen"] = isDoorOpen;
  statusJson["error"] = error;
  statusJson["state"] = stateStrings[state];
  return statusJson;
}
