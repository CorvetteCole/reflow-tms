#ifndef REFLOW_TMS_CONSTANTS_H
#define REFLOW_TMS_CONSTANTS_H

#define HEATING_ELEMENT_PWM_FREQUENCY 30.0f
#define MAX_TARGET_TEMPERATURE 300
#define MAX_TEMPERATURE 350

#define TOP_HEATING_ELEMENT_PIN 9
#define BOTTOM_HEATING_ELEMENT_PIN 10
#define DOOR_PIN 2
#define RESET_PIN 3
#define CS_PIN 14
#define DI_PIN 15
#define DO_PIN 16
#define CLK_PIN 17

#define TOP_HEATING_ELEMENT_KP 1.0f
#define TOP_HEATING_ELEMENT_KI 0.0f
#define TOP_HEATING_ELEMENT_KD 0.0f
#define BOTTOM_HEATING_ELEMENT_KP 1.0f
#define BOTTOM_HEATING_ELEMENT_KI 0.0f
#define BOTTOM_HEATING_ELEMENT_KD 0.0f

#define INPUT_BUFFER_SIZE 32

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

#define UI_TIMEOUT 250 // in milliseconds
#define STATUS_SEND_INTERVAL 100 // ms

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
