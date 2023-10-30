#include <ArduinoJson.h>

#ifndef REFLOW_TMS_STATUS_H
#define REFLOW_TMS_STATUS_H

enum State { IDLE, HEATING, COOLING, ERROR };

class Status {
public:
  uint16_t targetTemperature{}; // in degrees Celsius
  uint16_t currentTemperature{};
  uint8_t topHeatDutyCycle{};
  uint8_t bottomHeatDutyCycle{};
  bool isDoorOpen = false;
  State state = IDLE;

  StaticJsonDocument<192> toJson() const volatile;
};

#endif // REFLOW_TMS_STATUS_H
