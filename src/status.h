#include <ArduinoJson.h>

#ifndef REFLOW_TMS_STATUS_H
#define REFLOW_TMS_STATUS_H

enum State { IDLE, HEATING, COOLING, ERROR };

class Status {
public:
  uint8_t targetTemperature{}; // in degrees Celsius
  uint8_t currentTemperature{};
  uint8_t topHeatDutyCycle{};
  uint8_t bottomHeatDutyCycle{};
  bool isDoorOpen = false;
  char *error = nullptr;
  State state = IDLE;

  DynamicJsonDocument toJson() const volatile;
};

#endif // REFLOW_TMS_STATUS_H
