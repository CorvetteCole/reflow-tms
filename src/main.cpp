#include <Arduino.h>
#include <ArduinoJson.h>
#include <QuickPID.h>
#include <SimpleTimer.h>
#define USE_TIMER_1 true
#define USING_MICROS_RESOLUTION true

#define _PWM_LOGLEVEL_ 3
#include "Status.h"
#include "constants.h"
#include <AVR_Slow_PWM.h>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_FREQ 10000L

AVR_Slow_PWM heatingElementPwm;

volatile Status status;

float pidCurrentTemperature, pidTargetTemperature, pidTopHeatDutyCycle,
    pidBottomHeatDutyCycle;

QuickPID topHeatingElementPid(&pidCurrentTemperature, &pidTopHeatDutyCycle,
                              &pidTargetTemperature);

QuickPID bottomHeatingElementPid(&pidCurrentTemperature,
                                 &pidBottomHeatDutyCycle,
                                 &pidTargetTemperature);

void timerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
                                     heatingElementPwmFrequency, 0);
  heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
                                     heatingElementPwmFrequency, 0);
  heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN, heatingElementPwmFrequency,
                                     0);
  heatingElementPwm.disableAll();

  status.topHeatDutyCycle = 0;
  status.bottomHeatDutyCycle = 0;
}

void doorChanged() {
  status.isDoorOpen = digitalRead(doorPin);
  Serial.print("Door open: ");
  Serial.println(status.isDoorOpen);

  if (status.state == heating && status.isDoorOpen) {
    strcpy(status.error, "doorOpenedDuringHeating");
    status.state = error;
    immediateStop();
  }
}

void sendStatus() {
  // TODO attach to timer
  serializeJson(status.toJson(), Serial);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // wait for serial to connect

  Serial.println("\nStarting thermal management system...");
  Serial.println(BOARD_NAME);
  Serial.println(AVR_SLOW_PWM_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

  ITimer1.init();

  if (ITimer1.attachInterrupt(HW_TIMER_INTERVAL_FREQ, timerHandler)) {
    Serial.println(F("Starting  ITimer1 OK"));
    Serial.println(micros());
  } else {
    Serial.println(
        F("Can't set ITimer1 correctly. Select another freq. or timer"));
  }

  // initialize built-in LED pin as an output (will blink on heartbeat)
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize door sensor pin as an input
  pinMode(doorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(doorPin), doorChanged, CHANGE);

  // initialize temperature sensor pin as an input
  pinMode(temperatureSensorPin, INPUT);

  // initialize both heating element PWM interfaces, set duty cycle to 0
  heatingElementPwm.setPWM(heatingElementPwmPins[0], heatingElementPwmFrequency,
                           0);
  heatingElementPwm.setPWM(heatingElementPwmPins[1], heatingElementPwmFrequency,
                           0);
  heatingElementPwm.setPWM(LED_BUILTIN, heatingElementPwmFrequency, 0);

  heatingElementPwm.disableAll(); // disable timers while we aren't using them

  topHeatingElementPid.SetOutputLimits(0, 100);
  topHeatingElementPid.SetTunings(topHeatingElementKp, topHeatingElementKi,
                                  topHeatingElementKd);

  bottomHeatingElementPid.SetOutputLimits(0, 100);
  bottomHeatingElementPid.SetTunings(
      bottomHeatingElementKp, bottomHeatingElementKi, bottomHeatingElementKd);

  // TODO setup timer for sending status
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
bool readTemperature() {
  bool success = true;

  // read temperature sensor
  auto temperatureSensorReading = analogRead(temperatureSensorPin);
  // TODO do math to convert reading to degrees Celsius

  // TODO check if reading was reasonable

  //  status.currentTemperature = temperatureSensorReading;

  return true;
}

uint8_t lastTopHeatDutyCycle = 0;
uint8_t lastBottomHeatDutyCycle = 0;

void loop() {
  readTemperature();
  if (status.state == State::error) {
    // make sure heating elements are off
    if (status.topHeatDutyCycle != 0 || status.bottomHeatDutyCycle != 0) {
      Serial.println(
          "Heating elements should be off already! Turning off now...");
      immediateStop();
    }

    // TODO need to wait for reset command
    return;
  }

  if (status.targetTemperature == 0) {
    status.state = idle;
    topHeatingElementPid.SetMode(QuickPID::Control::manual);
    topHeatingElementPid.Reset();
    bottomHeatingElementPid.SetMode(QuickPID::Control::manual);
    bottomHeatingElementPid.Reset();
    status.topHeatDutyCycle = 0;
    status.bottomHeatDutyCycle = 0;
    heatingElementPwm.disableAll(); // disable timers while we aren't using them
  } else if (status.currentTemperature > status.targetTemperature &&
             status.currentTemperature - status.targetTemperature > 5) {
    status.state = cooling;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  } else if (status.targetTemperature > status.currentTemperature &&
             status.targetTemperature - status.currentTemperature > 5) {
    status.state = heating;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  }

  if (status.state != idle) {
    heatingElementPwm.enableAll();
    pidTargetTemperature = status.targetTemperature;
    pidCurrentTemperature = status.currentTemperature;
    topHeatingElementPid.Compute();
    bottomHeatingElementPid.Compute();
    // we can statically cast to uint8_t because the output limits are set to
    // 0-100
    status.topHeatDutyCycle = static_cast<uint8_t>(pidTopHeatDutyCycle);
    status.bottomHeatDutyCycle = static_cast<uint8_t>(pidBottomHeatDutyCycle);
  }

  if (status.topHeatDutyCycle != lastTopHeatDutyCycle ||
      status.bottomHeatDutyCycle != lastBottomHeatDutyCycle) {
    // set PWM
    heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
                                       heatingElementPwmFrequency,
                                       status.topHeatDutyCycle);
    heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
                                       heatingElementPwmFrequency,
                                       status.bottomHeatDutyCycle);
    heatingElementPwm.modifyPWMChannel(
        2, LED_BUILTIN, heatingElementPwmFrequency, status.bottomHeatDutyCycle);
    lastTopHeatDutyCycle = status.topHeatDutyCycle;
    lastBottomHeatDutyCycle = status.bottomHeatDutyCycle;
  }
}

// void loop() {
//   heatingElementPwm.enableAll();
//
//   for (int i = 0; i <= 100; i++) {
//     Serial.print("Setting heating elements PWM duty cycle to ");
//     Serial.print(i);
//     Serial.println("%");
//     heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
//                                        heatingElementPwmFrequency, i);
//     heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
//                                        heatingElementPwmFrequency, i);
//     heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
//                                        heatingElementPwmFrequency, i);
//     delay(100);
//   }
//   delay(5000);
//   Serial.println("Disabling all heating elements");
//   heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
//                                      heatingElementPwmFrequency, 0);
//   heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
//                                      heatingElementPwmFrequency, 0);
//   heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
//   heatingElementPwmFrequency,
//                                      0);
//   heatingElementPwm.disableAll();
//   delay(5000);
// }