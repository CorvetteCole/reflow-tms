#include <Arduino.h>
#include <ArduinoJson.h>

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

AVR_Slow_PWM heatingElementPwm;

Status status;

Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN, DI_PIN, DO_PIN, CLK_PIN);

Logger logger = Logger(LogLevel::CRITICAL);

void pwmTimerHandler() { heatingElementPwm.run(); }

uint8_t lastHeatDutyCycle = 0;
uint32_t lastSentPidTunerStatus = 0;

// target temperatures used to create step response
static const int targetTemperatureCurvePoints[] = {25,  100, 150, 183,
                                                   235, 183, 150};

static const int numTargetTemperatureCurvePoints =
    sizeof(targetTemperatureCurvePoints) /
    sizeof(targetTemperatureCurvePoints[0]);

int targetTemperatureCurveIndex = 0;

// max difference between last recorded points before the temperature is
// considered stable
static const float stabilizedThreshold = 0.5f;

// number of points to consider when determining if the temperature is stable
static const int stabilizationWindow = 4;

// allowable difference between target and current temperature to be considered
// "complete"
static const float targetTemperatureAchievedThreshold = 2;

// use PID_TUNER_SEND_INTERVAL as timing between points

// float previousTemperaturePoints[stabilizationWindow] = {0};

// circular buffer of previous temperature points
float previousTemperaturePoints[stabilizationWindow] = {0};

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

void setup() {
  Serial.begin(115200);

  // wait for serial port to connect
  while (!Serial || !Serial.available()) {
    delay(100);
  }

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

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865_3WIRE);

  logger.info(F("Thermal management system started"));

  logger.warn("starting in 3 seconds...");
  readTemperature();
  status.targetTemperature =
      targetTemperatureCurvePoints[targetTemperatureCurveIndex];
  delay(3000);
}

void loop() {

  // send status
  if (millis() - lastSentPidTunerStatus > PID_TUNER_SEND_INTERVAL) {
    // record current temperature to buffer
    for (int i = 0; i < stabilizationWindow - 1; i++) {
      previousTemperaturePoints[i] = previousTemperaturePoints[i + 1];
    }
    previousTemperaturePoints[stabilizationWindow - 1] =
        status.currentTemperature;

    readTemperature();

    // send time, current duty cycle, and current temperature
    // time should be in seconds with decimal
    Serial.print(millis() / 1000.0f);
    Serial.print(",");
    Serial.print(status.heatDutyCycle);
    Serial.print(",");
    Serial.println(status.currentTemperature);
    lastSentPidTunerStatus = millis();
  }

  if (status.state == State::FAULT) {
    // if we're in a fault state, don't do anything
    return;
  }

  status.isDoorOpen = !digitalRead(DOOR_PIN);
  if (status.state == State::HEATING && status.isDoorOpen) {
    enterErrorState(ERROR_DOOR_OPENED_DURING_HEATING);
  }

  if (targetTemperatureCurveIndex < numTargetTemperatureCurvePoints) {
    if (status.state == State::IDLE) {
      // calculate whether the temperature is stable
      bool isTemperatureStable = true;
      for (int i = 0; i < stabilizationWindow - 1; i++) {
        if (abs(previousTemperaturePoints[i] -
                previousTemperaturePoints[i + 1]) > stabilizedThreshold) {
          isTemperatureStable = false;
          break;
        }
      }

      bool isWithinThreshold =
          abs(status.currentTemperature - status.targetTemperature) <=
          targetTemperatureAchievedThreshold;

      if (isTemperatureStable && isWithinThreshold) {
        targetTemperatureCurveIndex++;
        status.targetTemperature =
            targetTemperatureCurvePoints[targetTemperatureCurveIndex];

        // determine if we should be heating
        if (status.targetTemperature > status.currentTemperature) {
          status.state = State::HEATING;
        }
      }
    } else if (status.state == State::HEATING) {
      if (status.currentTemperature < status.targetTemperature) {
        status.heatDutyCycle = 100;
      } else {
        status.state = State::IDLE;
        status.heatDutyCycle = 0;
      }
    }
  } else {
    status.state = State::IDLE;
    status.heatDutyCycle = 0;
  }

  if (status.heatDutyCycle != lastHeatDutyCycle) {
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
}
