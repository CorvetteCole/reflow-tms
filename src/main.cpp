#include <Arduino.h>
#include <ArduinoJson.h>
#include <QuickPID.h>
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

float pidCurrentTemperature, pidTargetTemperature, pidTopHeatDutyCycle,
    pidBottomHeatDutyCycle;

// QuickPID topHeatingElementPid(&pidCurrentTemperature, &pidTopHeatDutyCycle,
//                               &pidTargetTemperature);

QuickPID bottomHeatingElementPid(&pidCurrentTemperature,
                                 &pidBottomHeatDutyCycle,
                                 &pidTargetTemperature);

Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN, DI_PIN, DO_PIN, CLK_PIN);

Logger logger = Logger(LogLevel::DEBUG);

void pwmTimerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(2, LED_BUILTIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  //  heatingElementPwm.disableAll();

  status.topHeatDutyCycle = 0;
  status.bottomHeatDutyCycle = 0;
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

void (*resetFunc)() = nullptr;

void setup() {
  Serial.begin(230400);
  while (!Serial && !Serial.available()) {
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

  //  topHeatingElementPid.SetOutputLimits(0, 100);
  //  topHeatingElementPid.SetTunings(
  //      TOP_HEATING_ELEMENT_KP, TOP_HEATING_ELEMENT_KI,
  //      TOP_HEATING_ELEMENT_KD);

  bottomHeatingElementPid.SetOutputLimits(0, 100);
  bottomHeatingElementPid.SetTunings(BOTTOM_HEATING_ELEMENT_KP,
                                     BOTTOM_HEATING_ELEMENT_KI,
                                     BOTTOM_HEATING_ELEMENT_KD);

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865_3WIRE);

  delay(1000);

  logger.info(F("Thermal management system started"));
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
void readTemperature() {
  uint16_t rtd = max31865.readRTD();
  float temperature = max31865.temperature(RNOMINAL, RREF);
  // format strings and print debug
  //  float ratio = rtd;
  //  ratio /= 32768;

  //    logger.debug((String("RTD value: ") + String(rtd, 8)).c_str());
  //  logger.debug(String("Ratio: ") + String(ratio, 8));
  //  logger.debug(String("Resistance: ") + String(RREF * ratio, 8));
  //  delay(100);
  //  logger.debug(String("Temperature: ") + String(temperature, 8));

  uint8_t fault = max31865.readFault();
  if (fault) {
    enterErrorState(ERROR_CURRENT_TEMPERATURE_FAULT);

    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      logger.error(F("RTD High Threshold"));
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      logger.error(F("RTD Low Threshold"));
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      logger.error(F("REFIN- > 0.85 x Bias"));
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      logger.error(F("REFIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      logger.error(F("RTDIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_OVUV) {
      logger.error(F("Under/Over voltage"));
    }
    delay(100);
  }

  status.currentTemperature = static_cast<uint16_t>(temperature);

  if (status.currentTemperature > MAX_TEMPERATURE) {
    // if current temperature is over our hardcoded safety limit
    enterErrorState(ERROR_CURRENT_TEMPERATURE_TOO_HIGH);
  }
}

char receivedChars[INPUT_BUFFER_SIZE];
bool newData = false;

bool receiveCommand() {
  static bool receiveInProgress = false;

  static byte ndx = 0;
  char startMarker = '{';
  char endMarker = '}';
  char rc;

  while (Serial.available() > 0 && !newData) {
    rc = Serial.read();

    if (receiveInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= INPUT_BUFFER_SIZE) {
          ndx = INPUT_BUFFER_SIZE - 1;
        }
      } else {
        receivedChars[ndx] = endMarker;
        ndx++;
        receivedChars[ndx] = '\0'; // terminate the string
        receiveInProgress = false;
        ndx = 0;
        return true;
      }
    } else if (rc == startMarker) {
      receivedChars[ndx] = startMarker;
      ndx++;
      receiveInProgress = true;
    }
  }
  return false;
}

uint8_t lastTopHeatDutyCycle = 0;
uint8_t lastBottomHeatDutyCycle = 0;
uint32_t lastUiHeartbeat = 0;
uint32_t lastSentStatus = 0;

// TODO:
// - figure out why erroring doesn't actually set the output to off every time
// (!!)
// - figure out the funkiness with the pid loop integral which seems to get
// weird if you go between idle and heating states many times
// - figure out possible memory issues which cause a crash on error after awhile
// (I'm leaking memory somewhere....)

void loop() {
  static StaticJsonDocument<32> commandJson;

  status.isDoorOpen = digitalRead(DOOR_PIN);
  if (status.state == State::HEATING && status.isDoorOpen) {
    enterErrorState(ERROR_DOOR_OPENED_DURING_HEATING);
  }
  //  logger.debug(F("Reading temperature"));
  readTemperature();

  //  logger.debug(F("Reading command"));
  if (receiveCommand()) {
    lastUiHeartbeat = millis();

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(commandJson, receivedChars);

    // Test if parsing succeeds.
    if (error) {
      logger.warn(F("Could not parse command"));
      logger.warn(error.c_str());
    } else {
      bool commandPresent = false;

      if (commandJson.containsKey("targetTemperature")) {
        commandPresent = true;
        int targetTemperature = commandJson["targetTemperature"];
        if (targetTemperature < 0 ||
            targetTemperature > MAX_TARGET_TEMPERATURE) {
          logger.warn(F("Invalid target temperature"));
        } else {
          status.targetTemperature = targetTemperature;
        }
      }

      if (commandJson.containsKey("logLevel")) {
        commandPresent = true;
        const char *logLevel = commandJson["logLevel"];
        if (strcasecmp(logLevel, "DEBUG") == 0) {
          logger.logLevel = LogLevel::DEBUG;
        } else if (strcasecmp(logLevel, "INFO") == 0) {
          logger.logLevel = LogLevel::INFO;
        } else if (strcasecmp(logLevel, "WARN") == 0) {
          logger.logLevel = LogLevel::WARN;
        } else if (strcasecmp(logLevel, "CRITICAL") == 0) {
          logger.logLevel = LogLevel::CRITICAL;
        } else {
          logger.warn(F("Invalid log level"));
        }
      }

      if (!commandPresent) {
        logger.warn(F("No command present, received: "));
        serializeJson(commandJson, Serial);
        Serial.println();
      }

      newData = false;
    }
  } else if (lastUiHeartbeat != 0 && millis() - lastUiHeartbeat > UI_TIMEOUT) {
    enterErrorState(ERROR_UI_TIMEOUT);
  }

  // send status
  if (millis() - lastSentStatus > STATUS_SEND_INTERVAL) {
    sendStatus();
    lastSentStatus = millis();
  }

  if (status.state == State::FAULT) {
    //    logger.debug(F("In error state, not controlling heating elements"));
    // make sure HEATING elements are off
    if (status.topHeatDutyCycle != 0 || status.bottomHeatDutyCycle != 0) {
      logger.warn("Heating elements should be off already! Turning off now...");
      immediateStop();
      delay(500);
    }
    return;
  }

  if (status.targetTemperature == 0 && status.state != State::IDLE) {
    logger.debug(F("Target temperature is 0, resetting PID loop and disabling "
                   "heating elements"));
    status.state = State::IDLE;
    //    topHeatingElementPid.SetMode(QuickPID::Control::manual);
    //    topHeatingElementPid.Reset();
    bottomHeatingElementPid.SetMode(QuickPID::Control::manual);
    bottomHeatingElementPid.Reset();
    status.topHeatDutyCycle = 0;
    status.bottomHeatDutyCycle = 0;
    //    heatingElementPwm.disableAll(); // disable timers while we aren't
    //    using them
  } else if (status.state != State::COOLING && status.targetTemperature != 0 &&
             status.currentTemperature > status.targetTemperature) {
    logger.info(F("Started cooling"));
    status.state = State::COOLING;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  } else if (status.state != State::HEATING && status.targetTemperature != 0 &&
             status.targetTemperature > status.currentTemperature) {
    logger.info(F("Started heating"));
    status.state = State::HEATING;
    bottomHeatingElementPid.SetMode(QuickPID::Control::automatic);
  }

  if (status.state != State::IDLE) {
    //    logger.debug(F("Calculating duty cycles"));
    //    heatingElementPwm.enableAll();
    pidTargetTemperature = status.targetTemperature;
    pidCurrentTemperature = status.currentTemperature;
    //    topHeatingElementPid.Compute();
    bottomHeatingElementPid.Compute();
    // we can statically cast to uint8_t because the output limits are set to
    // 0-100
    //    status.topHeatDutyCycle = static_cast<uint8_t>(pidTopHeatDutyCycle);
    status.topHeatDutyCycle = static_cast<uint8_t>(pidBottomHeatDutyCycle);
    status.bottomHeatDutyCycle = static_cast<uint8_t>(pidBottomHeatDutyCycle);
  }

  if (status.topHeatDutyCycle != lastTopHeatDutyCycle ||
      status.bottomHeatDutyCycle != lastBottomHeatDutyCycle) {
    //    logger.debug(F("Updating PWM"));
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
