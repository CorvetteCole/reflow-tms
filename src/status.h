#include <ArduinoJson.h>

#ifndef REFLOW_TMS_STATUS_H
#define REFLOW_TMS_STATUS_H

enum State { IDLE, HEATING, COOLING, FAULT };

class Status {
public:
  float currentTemperature{};
  uint8_t heatDutyCycle{};
  bool isDoorOpen = false;
  State state = State::IDLE;
  uint8_t error = 0;

  StaticJsonDocument<128> toJson() const;

private:
  const char *const stateStrings[4] = {"idle", "heating", "cooling", "fault"};
};

#endif // REFLOW_TMS_STATUS_H
