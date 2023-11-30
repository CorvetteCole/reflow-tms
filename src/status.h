#ifndef REFLOW_TMS_STATUS_H
#define REFLOW_TMS_STATUS_H

#include <stdint.h>

enum State { IDLE, HEATING, COOLING, FAULT };

class Status {
public:
  float targetTemperature{}; // in degrees Celsius
  float currentTemperature{};
  uint8_t topHeatDutyCycle{};
  uint8_t bottomHeatDutyCycle{};
  bool isDoorOpen = false;
  State state = State::IDLE;
  uint8_t error = 0;

  void print() const;

private:
  const char *const stateStrings[4] = {"idle", "heating", "cooling", "fault"};
};

#endif // REFLOW_TMS_STATUS_H
