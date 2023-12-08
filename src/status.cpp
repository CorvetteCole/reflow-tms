#include "status.h"

StaticJsonDocument<128> Status::toJson() const {
  StaticJsonDocument<128> statusJson;
  statusJson[F("current")] = currentTemperature;
  statusJson[F("state")] = stateStrings[state];
  statusJson[F("pwm")] = heatDutyCycle;
  statusJson[F("door")] = isDoorOpen ? F("open") : F("closed");
  statusJson[F("error")] = error;
  return statusJson;
}
