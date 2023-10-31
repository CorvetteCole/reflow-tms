#include <ArduinoJson.h>

#ifndef REFLOW_TMS_STATUS_H
#define REFLOW_TMS_STATUS_H

enum State { IDLE, HEATING, COOLING, FAULT };

class Status {
public:
  uint16_t targetTemperature{}; // in degrees Celsius
  uint16_t currentTemperature{};
  uint8_t topHeatDutyCycle{};
  uint8_t bottomHeatDutyCycle{};
  bool isDoorOpen = false;
  State state = State::IDLE;
  uint8_t error = 0;

  StaticJsonDocument<96> toJson() const;

private:
  const char *const stateStrings[4] = {"IDLE", "HEATING", "COOLING", "FAULT"};
};

#endif // REFLOW_TMS_STATUS_H
