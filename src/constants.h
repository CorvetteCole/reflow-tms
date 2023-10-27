#ifndef REFLOW_TMS_CONSTANTS_H
#define REFLOW_TMS_CONSTANTS_H

#define heatingElementPwmFrequency 30.0f
#define maxTemperature 400

#define topHeatingElementPin 9
#define bottomHeatingElementPin 10
#define doorPin 2
#define resetPin 3
#define csPin 14
#define diPin 15
#define doPin 16
#define clkPin 17

#define topHeatingElementKp 1.0f
#define topHeatingElementKi 0.0f
#define topHeatingElementKd 0.0f
#define bottomHeatingElementKp 1.0f
#define bottomHeatingElementKi 0.0f
#define bottomHeatingElementKd 0.0f

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

#define uiTimeout 250 // in milliseconds

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
