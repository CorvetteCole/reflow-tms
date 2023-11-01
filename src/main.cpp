#include <Arduino.h>
#include <ArduinoJson.h>
#include <QuickPID.h>
#include <sTune.h>
#define USE_TIMER_1 true
#define USING_MICROS_RESOLUTION true

#define _PWM_LOGLEVEL_ 0
#include "constants.h"
#include "logger.h"
#include "status.h"
#include "utils.cpp"
#include <AVR_Slow_PWM.h>
#include <Adafruit_MAX31865.h>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_FREQ 10000L

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500; // runPid interval = testTimeSec / samples
const uint16_t samples = 500;
const float inputSpan = 200;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 300;
uint8_t debounce = 1;
bool startup = true;

AVR_Slow_PWM heatingElementPwm;

Status status;

float pidCurrentTemperature, pidTargetTemperature = 130, pidHeatDutyCycle = 0,
                             Kp, Ki, Kd;

sTune tuner = sTune(&pidCurrentTemperature, &pidHeatDutyCycle, sTune::ZN_PID,
                    sTune::directIP, sTune::printOFF);
QuickPID heatingElementPid(&pidCurrentTemperature, &pidHeatDutyCycle,
                           &pidTargetTemperature);

Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN, DI_PIN, DO_PIN, CLK_PIN);

Logger logger = Logger(LogLevel::CRITICAL);

void pwmTimerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  //  heatingElementPwm.disableAll();

  status.heatDutyCycle = 0;
}

void sendStatus() {
  serializeJson(status.toJson(), Serial);
  Serial.println();
}

void enterErrorState(uint8_t error) {
  if (status.state != State::FAULT) {
    immediateStop();
    status.state = State::FAULT;
  }
  if (!(status.error & error)) {
    // this is a new error!
    status.error |= error;
    logger.error(ovenErrorToString(error));
    sendStatus(); // immediately update status
  }
}

void setup() {
  Serial.begin(115200);

  logger.info(F("Starting thermal management system..."));
  //  logger.info(BOARD_NAME);

  logger.debug(F("Initializing ITimer1..."));

  ITimer1.init();

  logger.debug(F("Attaching ITimer1 interrupt..."));

  if (ITimer1.attachInterrupt(HW_TIMER_INTERVAL_FREQ, pwmTimerHandler)) {
    logger.debug(F("Starting ITimer1 OK"));
  } else {
    logger.error(
        F("Can't set ITimer1 correctly. Select another freq. or timer"));
  }

  // initialize built-in LED pin as an output (will blink on heartbeat)
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(TOP_HEATING_ELEMENT_PIN, OUTPUT);
  pinMode(BOTTOM_HEATING_ELEMENT_PIN, OUTPUT);

  // initialize door sensor pin as an input
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);

  logger.debug(F("Attaching interrupts..."));

  //  attachInterrupt(digitalPinToInterrupt(RESET_PIN), resetFunc, FALLING);

  logger.debug(F("Initializing PWM..."));

  // initialize both HEATING element PWM interfaces, set duty cycle to 0
  heatingElementPwm.setPWM(TOP_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.setPWM(BOTTOM_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.setPWM(LED_BUILTIN, HEATING_ELEMENT_PWM_FREQUENCY, 0);

  //  heatingElementPwm.disableAll(); // disable timers while we aren't using
  //  them

  logger.debug(F("Initializing PID..."));

  //  heatingElementPid.SetOutputLimits(0, 100);
  //  heatingElementPid.SetTunings(TOP_HEATING_ELEMENT_KP,
  //  TOP_HEATING_ELEMENT_KI,
  //                               TOP_HEATING_ELEMENT_KD);

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865_3WIRE);

  logger.info(F("Thermal management system started"));

  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);

  delay(3000);
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
void readTemperature() {
  uint16_t rtd = max31865.readRTD();
  float temperature = max31865.temperature(RNOMINAL, RREF);

  if (logger.logLevel == LogLevel::DEBUG) {
    // format strings and print debug
    float ratio = rtd;
    ratio /= 32768;

    logger.debug((String("Resistance: ") + String(RREF * ratio, 8)).c_str());
  }

  uint8_t fault = max31865.readFault();
  if (fault) {
    enterErrorState(ERROR_CURRENT_TEMPERATURE_FAULT);

    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      logger.debug(F("RTD High Threshold"));
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      logger.debug(F("RTD Low Threshold"));
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      logger.debug(F("REFIN- > 0.85 x Bias"));
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      logger.debug(F("REFIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      logger.debug(F("RTDIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_OVUV) {
      logger.debug(F("Under/Over voltage"));
    }
    delay(100);
  }
  status.currentTemperature = temperature;

  if (temperature > MAX_TEMPERATURE) {
    // if current temperature is over our hardcoded safety limit
    enterErrorState(ERROR_CURRENT_TEMPERATURE_TOO_HIGH);
  } else if (temperature < MIN_TEMPERATURE) {
    // temperature is an unsigned int, so this is a sign that something is wrong
    enterErrorState(ERROR_CURRENT_TEMPERATURE_TOO_LOW);
  }
}

uint8_t lastHeatDutyCycle = 0;

void loop() {
  if (status.state == State::FAULT) {
    // if we're in a fault state, don't do anything
    return;
  }

  status.isDoorOpen = !digitalRead(DOOR_PIN);
  if (status.state == State::HEATING && status.isDoorOpen) {
    enterErrorState(ERROR_DOOR_OPENED_DURING_HEATING);
  }

  switch (tuner.Run()) {
  case sTune::sample:
    readTemperature();
    pidCurrentTemperature = status.currentTemperature;
    tuner.plotter(pidCurrentTemperature, pidHeatDutyCycle, pidTargetTemperature,
                  0.5f, 3);
    break;
  case sTune::tunings:
    tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
    heatingElementPid.SetOutputLimits(0, outputSpan * 0.1);
    heatingElementPid.SetSampleTimeUs((outputSpan - 1) * 1000);
    debounce = 0; // ssr mode
    pidHeatDutyCycle = outputStep;
    heatingElementPid.SetMode(
        QuickPID::Control::automatic); // the PID is turned on
    heatingElementPid.SetProportionalMode(QuickPID::pMode::pOnMeas);
    heatingElementPid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    heatingElementPid.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
    break;

  case sTune::runPid:
    if (startup &&
        pidCurrentTemperature > pidTargetTemperature - 5) { // reduce overshoot
      startup = false;
      pidHeatDutyCycle -= 9;
      heatingElementPid.SetMode(QuickPID::Control::manual);
      heatingElementPid.SetMode(QuickPID::Control::automatic);
    }
    readTemperature();
    pidCurrentTemperature = status.currentTemperature;
    heatingElementPid.Compute();
    tuner.plotter(pidCurrentTemperature, pidHeatDutyCycle, pidTargetTemperature,
                  0.5f, 3);
    break;
  }

  if (status.heatDutyCycle != lastHeatDutyCycle) {
    //    logger.debug(F("Updating PWM"));
    // set PWM
    heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.heatDutyCycle);
    heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.heatDutyCycle);
    heatingElementPwm.modifyPWMChannel(
        2, LED_BUILTIN, HEATING_ELEMENT_PWM_FREQUENCY, status.heatDutyCycle);
    lastHeatDutyCycle = status.heatDutyCycle;
  }

  // send status
//  if (millis() - lastSentStatus > STATUS_SEND_INTERVAL) {
//    sendStatus();
//    lastSentStatus = millis();
//  }
}
