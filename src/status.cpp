#include "status.h"

StaticJsonDocument<96> Status::toJson() const {
  StaticJsonDocument<96> statusJson;
  statusJson[F("target")] = targetTemperature;
  statusJson[F("current")] = currentTemperature;
  statusJson[F("top")] = topHeatDutyCycle;
  statusJson[F("bottom")] = bottomHeatDutyCycle;
  statusJson[F("door_open")] = isDoorOpen;
  statusJson[F("error")] = error;
  statusJson[F("state")] = stateStrings[state];
  return statusJson;
}
