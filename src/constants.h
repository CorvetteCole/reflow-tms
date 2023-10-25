#include <Arduino.h>

#ifndef REFLOW_TMS_CONSTANTS_H
#define REFLOW_TMS_CONSTANTS_H

const uint32_t heatingElementPwmPins[] = {9, 10}; // top, bottom
const float heatingElementPwmFrequency = 30.0f;
const uint16_t maxTemperature = 400;

const uint8_t doorPin = 2;
const uint8_t temperatureSensorPin = A0;

float topHeatingElementKp = 1.0f;
float topHeatingElementKi = 0.0f;
float topHeatingElementKd = 0.0f;
float bottomHeatingElementKp = 1.0f;
float bottomHeatingElementKi = 0.0f;
float bottomHeatingElementKd = 0.0f;

int uiTimeout = 250; // in milliseconds

// enum Error {
//   doorOpenedDuringHeating,
//   targetTemperatureTooLow,
//   targetTemperatureTooHigh,
//   currentTemperatureTooHigh,
//   currentTemperatureTooLow,
//   currentTemperatureNotRisingDuringHeating,
//   currentTemperatureUnstable,
//   uiTimeout,
//   noError
// };

// typedef struct {
//   uint16_t targetTemperature; // in degrees Celsius
//   uint16_t currentTemperature;
//   uint8_t topHeatDutyCycle;
//   uint8_t bottomHeatDutyCycle;
//   bool isDoorOpen;
//   Error error;
//   State state;
//   CommunicationMode communicationMode;
// } Status;

#endif // REFLOW_TMS_CONSTANTS_H
