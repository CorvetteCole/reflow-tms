#include "status.h"

StaticJsonDocument<128> Status::toJson() const {
  StaticJsonDocument<128> statusJson;
  statusJson[F("target")] = targetTemperature;
  statusJson[F("current")] = currentTemperature;
  statusJson[F("state")] = stateStrings[state];
  statusJson[F("top")] = topHeatDutyCycle;
  statusJson[F("bottom")] = bottomHeatDutyCycle;
  statusJson[F("door_open")] = isDoorOpen;
  statusJson[F("error")] = error;
  return statusJson;
}
