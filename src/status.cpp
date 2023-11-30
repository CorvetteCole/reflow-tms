#include "status.h"
#include <HardwareSerial.h>

void Status::print() const {
  // build JSON without ArduinoJson
  Serial.print("{\"target\":");
  Serial.print(targetTemperature);
  Serial.print(",\"current\":");
  Serial.print(currentTemperature);
  Serial.print(R"(,"state":")");
  Serial.print(stateStrings[state]);
  Serial.print(R"(","top":)");
  Serial.print(topHeatDutyCycle);
  Serial.print(",\"bottom\":");
  Serial.print(bottomHeatDutyCycle);
  Serial.print(R"(,"door":")");
  Serial.print(isDoorOpen ? "open" : "closed");
  Serial.print(R"(","error":)");
  Serial.print(error);
  Serial.println("}");
}