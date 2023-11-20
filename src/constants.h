#ifndef REFLOW_TMS_CONSTANTS_H
#define REFLOW_TMS_CONSTANTS_H

// ######### DEBUG FLAGS #########
//#define DISABLE_FAULT_HANDLING
//#define DISABLE_HEATING
//#define DISABLE_BUZZER
#define DISABLE_UI_TIMEOUT
// ###############################

#define HEATING_ELEMENT_PWM_FREQUENCY 1.0f
#define MAX_TARGET_TEMPERATURE 250
#define MIN_TARGET_TEMPERATURE 30
#define MAX_TEMPERATURE 250
#define MIN_TEMPERATURE 0

#define TOP_HEATING_ELEMENT_PIN 9
#define BOTTOM_HEATING_ELEMENT_PIN 8

#define DOOR_PIN 2
#define FAN_PIN 4
#define BUZZER_PIN 5

#define TOP_HEATING_ELEMENT_KP 300.0f
#define TOP_HEATING_ELEMENT_KI 0.15f
#define TOP_HEATING_ELEMENT_KD 150.0f
#define BOTTOM_HEATING_ELEMENT_KP 300.0f
#define BOTTOM_HEATING_ELEMENT_KI 0.15f
#define BOTTOM_HEATING_ELEMENT_KD 150.0f

#define INPUT_BUFFER_SIZE 32

#define ALARM_FREQUENCY 4000     // Hz
#define ATTENTION_FREQUENCY 1000 // Hz

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

#define UI_STALE_THRESHOLD_MILLIS 1000
#define TEMPERATURE_STALE_THRESHOLD_MILLIS 250
#define LOOP_SLOW_THRESHOLD_MICROS 5000 // 5ms
#define FAN_DEBOUNCE_THRESHOLD_MILLIS 30000

#define STATUS_INTERVAL_MILLIS 500 // ms
#define PID_INTERVAL_MICROS 100000 // 100ms

#define FAN_ON_TEMPERATURE 35.0f // degrees Celsius

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
