#include <Arduino.h>
#include <ArduinoJson.h>
#include <QuickPID.h>
#include <SimpleTimer.h>
#define USE_TIMER_1 true
#define USING_MICROS_RESOLUTION true

#define _PWM_LOGLEVEL_ 3
#include "constants.h"
#include "logger.h"
#include "status.h"
#include <AVR_Slow_PWM.h>
#include <Adafruit_MAX31865.h>

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

Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN, DI_PIN, DO_PIN, CLK_PIN);

Logger logger = Logger(LogLevel::DEBUG);

void timerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.disableAll();

  status.topHeatDutyCycle = 0;
  status.bottomHeatDutyCycle = 0;
}

void sendStatus() {
  // TODO attach to timer
  serializeJson(status.toJson(), Serial);
}

void enterErrorState(const char *error) {
  strcpy(status.error, error);
  status.state = State::ERROR;
  logger.error(error);
  immediateStop();
  sendStatus(); // immediately update status
}

void doorChanged() {
  status.isDoorOpen = digitalRead(DOOR_PIN);
  Serial.print("Door open: ");
  Serial.println(status.isDoorOpen);

  if (status.state == HEATING && status.isDoorOpen) {
    enterErrorState("Door opened during HEATING");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && !Serial.available()) {
  }

  logger.info("Starting thermal management system...");
  logger.info(BOARD_NAME);

  ITimer1.init();

  if (ITimer1.attachInterrupt(HW_TIMER_INTERVAL_FREQ, timerHandler)) {
    logger.debug("Starting  ITimer1 OK");
  } else {
    logger.error("Can't set ITimer1 correctly. Select another freq. or timer");
  }

  // initialize built-in LED pin as an output (will blink on heartbeat)
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize door sensor pin as an input
  pinMode(DOOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DOOR_PIN), doorChanged, CHANGE);

  // initialize both HEATING element PWM interfaces, set duty cycle to 0
  heatingElementPwm.setPWM(TOP_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.setPWM(BOTTOM_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.setPWM(LED_BUILTIN, HEATING_ELEMENT_PWM_FREQUENCY, 0);

  heatingElementPwm.disableAll(); // disable timers while we aren't using them

  topHeatingElementPid.SetOutputLimits(0, 100);
  topHeatingElementPid.SetTunings(
      TOP_HEATING_ELEMENT_KP, TOP_HEATING_ELEMENT_KI, TOP_HEATING_ELEMENT_KD);

  bottomHeatingElementPid.SetOutputLimits(0, 100);
  bottomHeatingElementPid.SetTunings(BOTTOM_HEATING_ELEMENT_KP,
                                     BOTTOM_HEATING_ELEMENT_KI,
                                     BOTTOM_HEATING_ELEMENT_KD);

  max31865.begin(MAX31865_3WIRE);

  logger.info("Thermal management system started");

  // TODO setup timer for sending status
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
void readTemperature() {
  max31865.temperature(RNOMINAL, RREF);
  uint8_t fault = max31865.readFault();
  if (fault) {
    // TODO do more?
    enterErrorState("Fault when reading temperature sensor");
  }

  // TODO check if reading was reasonable
}

uint8_t lastTopHeatDutyCycle = 0;
uint8_t lastBottomHeatDutyCycle = 0;

void loop() {
  readTemperature();
  if (status.state == State::ERROR) {
    // make sure HEATING elements are off
    if (status.topHeatDutyCycle != 0 || status.bottomHeatDutyCycle != 0) {
      logger.warn(
          "Heating elements should be off already! Turning off now...");
      immediateStop();
    }

    // TODO need to wait for reset command, clear fault on MAX31865 if present
    return;
  }

  if (status.targetTemperature == 0) {
    status.state = IDLE;
    topHeatingElementPid.SetMode(QuickPID::Control::manual);
    topHeatingElementPid.Reset();
    bottomHeatingElementPid.SetMode(QuickPID::Control::manual);
    bottomHeatingElementPid.Reset();
    status.topHeatDutyCycle = 0;
    status.bottomHeatDutyCycle = 0;
    heatingElementPwm.disableAll(); // disable timers while we aren't using them
  } else if (status.currentTemperature > status.targetTemperature &&
             status.currentTemperature - status.targetTemperature > 5) {
    status.state = COOLING;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  } else if (status.targetTemperature > status.currentTemperature &&
             status.targetTemperature - status.currentTemperature > 5) {
    status.state = HEATING;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  }

  if (status.state != IDLE) {
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
    heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.topHeatDutyCycle);
    heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.bottomHeatDutyCycle);
    heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.bottomHeatDutyCycle);
    lastTopHeatDutyCycle = status.topHeatDutyCycle;
    lastBottomHeatDutyCycle = status.bottomHeatDutyCycle;
  }
}

// void loop() {
//   heatingElementPwm.enableAll();
//
//   for (int i = 0; i <= 100; i++) {
//     Serial.print("Setting HEATING elements PWM duty cycle to ");
//     Serial.print(i);
//     Serial.println("%");
//     heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
//                                        HEATING_ELEMENT_PWM_FREQUENCY, i);
//     heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
//                                        HEATING_ELEMENT_PWM_FREQUENCY, i);
//     heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
//                                        HEATING_ELEMENT_PWM_FREQUENCY, i);
//     delay(100);
//   }
//   delay(5000);
//   Serial.println("Disabling all HEATING elements");
//   heatingElementPwm.modifyPWMChannel(0, heatingElementPwmPins[0],
//                                      HEATING_ELEMENT_PWM_FREQUENCY, 0);
//   heatingElementPwm.modifyPWMChannel(1, heatingElementPwmPins[1],
//                                      HEATING_ELEMENT_PWM_FREQUENCY, 0);
//   heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
//   HEATING_ELEMENT_PWM_FREQUENCY,
//                                      0);
//   heatingElementPwm.disableAll();
//   delay(5000);
// }