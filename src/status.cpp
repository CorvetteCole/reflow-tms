#include "status.h"

StaticJsonDocument<192> Status::toJson() const {
  StaticJsonDocument<192> statusJson;
  statusJson["targetTemperature"] = targetTemperature;
  statusJson["currentTemperature"] = currentTemperature;
  statusJson["topHeatDutyCycle"] = topHeatDutyCycle;
  statusJson["bottomHeatDutyCycle"] = bottomHeatDutyCycle;
  statusJson["isDoorOpen"] = isDoorOpen;
  statusJson["error"] = error;
  statusJson["state"] = stateStrings[state];
  return statusJson;
}
