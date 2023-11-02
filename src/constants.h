#ifndef REFLOW_TMS_CONSTANTS_H
#define REFLOW_TMS_CONSTANTS_H

#define HEATING_ELEMENT_PWM_FREQUENCY 0.25f
#define MAX_TARGET_TEMPERATURE 300
#define MIN_TARGET_TEMPERATURE 30
#define MAX_TEMPERATURE 325
#define MIN_TEMPERATURE 0

#define TOP_HEATING_ELEMENT_PIN 9
#define BOTTOM_HEATING_ELEMENT_PIN 8

#define DOOR_PIN 2
#define RESET_PIN 3
#define FAN_PIN 4

#define TOP_HEATING_ELEMENT_KP 3.0f
#define TOP_HEATING_ELEMENT_KI 0.0f
#define TOP_HEATING_ELEMENT_KD 0.0f
#define BOTTOM_HEATING_ELEMENT_KP 3.0f
#define BOTTOM_HEATING_ELEMENT_KI 0.0f
#define BOTTOM_HEATING_ELEMENT_KD 0.0f

#define INPUT_BUFFER_SIZE 32

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

#define UI_TIMEOUT 1000          // in milliseconds
#define STATUS_SEND_INTERVAL 5000 // ms
#define LOOP_SLOW_THRESHOLD_MICROS 5000 // 5ms

// binary error codes (can be combined)
#define ERROR_DOOR_OPENED_DURING_HEATING 0x01
#define ERROR_TARGET_TEMPERATURE_TOO_LOW 0x02
#define ERROR_TARGET_TEMPERATURE_TOO_HIGH 0x04
#define ERROR_CURRENT_TEMPERATURE_TOO_LOW 0x08
#define ERROR_CURRENT_TEMPERATURE_TOO_HIGH 0x10
#define ERROR_CURRENT_TEMPERATURE_NOT_RISING_DURING_HEATING 0x20
#define ERROR_CURRENT_TEMPERATURE_FAULT 0x40
#define ERROR_UI_TIMEOUT 0x80

#endif // REFLOW_TMS_CONSTANTS_H
