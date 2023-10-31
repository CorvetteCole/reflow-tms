#include "constants.h"
#include <Arduino.h>

inline const __FlashStringHelper *ovenErrorToString(uint8_t error) {
  switch (error) {
  case ERROR_DOOR_OPENED_DURING_HEATING:
    return F("Door opened during heating");
  case ERROR_TARGET_TEMPERATURE_TOO_LOW:
    return F("Target temperature too low");
  case ERROR_TARGET_TEMPERATURE_TOO_HIGH:
    return F("Target temperature too high");
  case ERROR_CURRENT_TEMPERATURE_TOO_LOW:
    return F("Current temperature too low");
  case ERROR_CURRENT_TEMPERATURE_TOO_HIGH:
    return F("Current temperature too high");
  case ERROR_CURRENT_TEMPERATURE_NOT_RISING_DURING_HEATING:
    return F("Current temperature not rising during heating");
  case ERROR_CURRENT_TEMPERATURE_FAULT:
    return F("Fault when reading current temperature");
  case ERROR_UI_TIMEOUT:
    return F("UI timeout");
  default:
    return F("Unknown error");
  }
}