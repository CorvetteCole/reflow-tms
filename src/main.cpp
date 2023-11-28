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
#include <MAX31865_NonBlocking.h>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_FREQ 1000L

AVR_Slow_PWM heatingElementPwm;

Status status;

float pidCurrentTemperature, pidTargetTemperature, pidTopHeatDutyCycle,
    pidBottomHeatDutyCycle;

char receivedChars[INPUT_BUFFER_SIZE];
bool newData = false; // represents whether the receivedChars array is ready to
                      // be parsed

QuickPID topHeatingElementPid(&pidCurrentTemperature, &pidTopHeatDutyCycle,
                              &pidTargetTemperature);

QuickPID bottomHeatingElementPid(&pidCurrentTemperature,
                                 &pidBottomHeatDutyCycle,
                                 &pidTargetTemperature);

MAX31865 max31865(10);

Logger logger = Logger(LogLevel::INFO);

void pwmTimerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);

  status.topHeatDutyCycle = 0;
  status.bottomHeatDutyCycle = 0;
}

void sendStatus() {
  serializeJson(status.toJson(), Serial);
  Serial.println();
}

void enterErrorState(uint8_t error) {
  if (status.state != State::FAULT) {
#ifndef DISABLE_FAULT_HANDLING
#ifndef DISABLE_BUZZER
    tone(BUZZER_PIN, ALARM_FREQUENCY);
#endif
    immediateStop();
    status.state = State::FAULT;
#endif
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
  Serial.begin(115200);

  logger.info(F("Starting thermal management system..."));

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
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(TOP_HEATING_ELEMENT_PIN, OUTPUT);
  pinMode(BOTTOM_HEATING_ELEMENT_PIN, OUTPUT);

  // initialize door sensor pin as an input
  pinMode(DOOR_PIN, INPUT_PULLUP);

  logger.debug(F("Initializing PWM..."));

  // initialize both HEATING element PWM interfaces, set duty cycle to 0
  heatingElementPwm.setPWM(TOP_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.setPWM(BOTTOM_HEATING_ELEMENT_PIN,
                           HEATING_ELEMENT_PWM_FREQUENCY, 0);

  logger.debug(F("Initializing PID..."));

  topHeatingElementPid.SetOutputLimits(0, 100);
  topHeatingElementPid.SetSampleTimeUs(PID_INTERVAL_MICROS);
  topHeatingElementPid.SetTunings(
      TOP_HEATING_ELEMENT_KP, TOP_HEATING_ELEMENT_KI, TOP_HEATING_ELEMENT_KD);
  topHeatingElementPid.SetProportionalMode(QuickPID::pMode::pOnErrorMeas);
  topHeatingElementPid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);

  bottomHeatingElementPid.SetOutputLimits(0, 100);
  bottomHeatingElementPid.SetSampleTimeUs(PID_INTERVAL_MICROS);
  bottomHeatingElementPid.SetTunings(BOTTOM_HEATING_ELEMENT_KP,
                                     BOTTOM_HEATING_ELEMENT_KI,
                                     BOTTOM_HEATING_ELEMENT_KD);
  bottomHeatingElementPid.SetProportionalMode(QuickPID::pMode::pOnErrorMeas);
  bottomHeatingElementPid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865::RTD_3WIRE, MAX31865::FILTER_60HZ);

  status.state = State::IDLE;
  topHeatingElementPid.SetMode(QuickPID::Control::manual);
  topHeatingElementPid.Reset();
  bottomHeatingElementPid.SetMode(QuickPID::Control::manual);
  bottomHeatingElementPid.Reset();
  status.topHeatDutyCycle = 0;
  status.bottomHeatDutyCycle = 0;

  logger.info(F("Thermal management system started"));
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
void readTemperature() {
  static unsigned long lastTemperatureRead = 0;
  if (!max31865.isConversionComplete()) {
    // detect stale temperature readings
    if (lastTemperatureRead != 0 &&
        millis() - lastTemperatureRead > TEMPERATURE_STALE_THRESHOLD_MILLIS) {
      enterErrorState(ERROR_CURRENT_TEMPERATURE_FAULT);
    }
    return;
  }

  float temperature = max31865.getTemperature(RNOMINAL, RREF);

  uint8_t fault = max31865.getFault();
  if (fault) {
    enterErrorState(ERROR_CURRENT_TEMPERATURE_FAULT);

    if (fault & MAX31865::FAULT_HIGHTHRESH_BIT) {
      logger.debug(F("RTD High Threshold"));
    }
    if (fault & MAX31865::FAULT_LOWTHRESH_BIT) {
      logger.debug(F("RTD Low Threshold"));
    }
    if (fault & MAX31865::FAULT_REFINLOW_BIT) {
      logger.debug(F("REFIN- > 0.85 x Bias"));
    }
    if (fault & MAX31865::FAULT_REFINHIGH_BIT) {
      logger.debug(F("REFIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865::FAULT_RTDINLOW_BIT) {
      logger.debug(F("RTDIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865::FAULT_OVUV_BIT) {
      logger.debug(F("Under/Over voltage"));
    }
    max31865.clearFault();
    delay(100); // wait for a bit so we aren't
  }
  status.currentTemperature = temperature;

  if (temperature > MAX_TEMPERATURE) {
    // if current temperature is over our hardcoded safety limit
    enterErrorState(ERROR_CURRENT_TEMPERATURE_TOO_HIGH);
  } else if (temperature < MIN_TEMPERATURE) {
    // temperature is an unsigned int, so this is a sign that something is wrong
    enterErrorState(ERROR_CURRENT_TEMPERATURE_TOO_LOW);
  }
  lastTemperatureRead = millis();
}

void computePid() {
  static unsigned long lastPidCompute = 0;

  if (status.state != State::HEATING) {
    // only compute PID if we are heating, since it is expensive
    return;
  }

  // if we are past the PID interval, we should log a warning
  if (lastPidCompute != 0 &&
      micros() - lastPidCompute >
          PID_INTERVAL_MICROS + 1000) { // TODO do we need this +1000?
    logger.warn(F("PID loop interval too long!"));
  }

  pidTargetTemperature = status.targetTemperature;
  pidCurrentTemperature = status.currentTemperature;

  topHeatingElementPid.Compute();
  bottomHeatingElementPid.Compute();
  lastPidCompute = micros();

  if (pidTopHeatDutyCycle > 100) {
    logger.warn(F("PID top heat duty cycle > 100%!"));
    pidTopHeatDutyCycle = 100;
  } else if (pidTopHeatDutyCycle < 0) {
    logger.warn(F("PID top heat duty cycle < 0%!"));
    pidTopHeatDutyCycle = 0;
  }

  if (pidBottomHeatDutyCycle > 100) {
    logger.warn(F("PID bottom heat duty cycle > 100%!"));
    pidBottomHeatDutyCycle = 100;
  } else if (pidBottomHeatDutyCycle < 0) {
    logger.warn(F("PID bottom heat duty cycle < 0%!"));
    pidBottomHeatDutyCycle = 0;
  }

  // we can statically cast to uint8_t because the output limits are set
  // to 0-100
  status.topHeatDutyCycle = static_cast<uint8_t>(pidTopHeatDutyCycle);
  status.bottomHeatDutyCycle = static_cast<uint8_t>(pidBottomHeatDutyCycle);
}

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




// generate step response at power levels 10, 20, 40, 80, 100
// basically, set the duty cycles of top and bottom heating elements to the
// power level we are getting a step response for. The basic idea for this is
// the following:
// 1. if the current step response hasn't started yet, record the current
//    and wait for it to "stabilize" (i.e. over the last 1 second it doesn't
//    change TOO much.
// 2. Record this stable reference temperature and set the power level to the
//    current step response index. Set currentStepResponseStarted to true.
// 3. Wait 60 seconds, or until temperature reaches safety limit of 250. Then
//    set currentStepResponseStarted to false, and iterate the current step
//    response index.
int currentStepResponseIndex = 0;
bool currentStepResponseStarted = false;
const int numberOfSteps = 5;
const int powerLevels[numberOfSteps] = {100, 80, 40, 20, 10};
const unsigned long stabilizationPeriodMilliseconds = 30000;
const unsigned long stepDurationMilliseconds = 60000;
const float safetyTemperatureLimit = 250.0f;

// Variables for stabilizing temperature tracking
unsigned long responseStartTime;
unsigned long stabilizingStartTime;
float stableReferenceTemperature;
float lastTemperatureRead;
bool temperatureStabilized;

void generateStepResponse() {
  if (currentStepResponseIndex >= numberOfSteps) {
    // All steps have been completed
    status.topHeatDutyCycle = 0;
    status.bottomHeatDutyCycle = 0;
    status.state = State::IDLE;
    return;
  }

  unsigned long currentTime = millis();

  // If the step response hasn't started, begin the stabilization period
  if (!currentStepResponseStarted) {
    responseStartTime = currentTime;
    stabilizingStartTime = currentTime;
    stableReferenceTemperature = status.currentTemperature;
    lastTemperatureRead = stableReferenceTemperature;
    temperatureStabilized = false;
    logger.info((String("Starting step response for power level ") + String(powerLevels[currentStepResponseIndex]) + String("%")).c_str());
    currentStepResponseStarted = true;
  } else {
    // Check if temperature has stabilized by comparing against last read
    if (abs(status.currentTemperature - lastTemperatureRead) < THRESHOLD_TEMPERATURE_CHANGE_FOR_STABILITY &&
        currentTime - stabilizingStartTime > stabilizationPeriodMilliseconds) {
      temperatureStabilized = true;
    }

    lastTemperatureRead = status.currentTemperature;

    if (temperatureStabilized) {
      // Stabilization period over, set the power levels
      status.topHeatDutyCycle = powerLevels[currentStepResponseIndex];
      status.bottomHeatDutyCycle = powerLevels[currentStepResponseIndex];
      status.state = State::HEATING;
    }

    // Check if we've completed the step duration or if temperature exceeded safety limit
    if (currentTime - responseStartTime >= stepDurationMilliseconds ||
        status.currentTemperature > safetyTemperatureLimit) {
      // Finish step, prepare for next step
      status.topHeatDutyCycle = 0;
      status.bottomHeatDutyCycle = 0;
      currentStepResponseIndex++;
      currentStepResponseStarted = false;
      status.state = State::COOLING;
    }
  }
}


// TODO: detect current temperature not rising during heating
// TODO: implement CRC16 checksums for commands
void loop() {
  static uint8_t lastTopHeatDutyCycle = 0;
  static uint8_t lastBottomHeatDutyCycle = 0;
  static unsigned long lastUiHeartbeat = 0;
  static unsigned long lastSentStatus = 0;
  static unsigned long loopTime = 0;
  static unsigned long lastFanToggle = 0;
  static bool isFanOn = false;
  static StaticJsonDocument<32> commandJson;

  loopTime = micros();

  status.isDoorOpen = !digitalRead(DOOR_PIN);
  if (status.state == State::HEATING && status.isDoorOpen) {
    enterErrorState(ERROR_DOOR_OPENED_DURING_HEATING);
  }
  //  logger.debug(F("Reading temperature"));
  readTemperature();

  if (status.currentTemperature > FAN_ON_TEMPERATURE && !isFanOn &&
      millis() - lastFanToggle > FAN_DEBOUNCE_THRESHOLD_MILLIS) {
    logger.info(F("Turning on fan"));
    digitalWrite(FAN_PIN, HIGH);
    isFanOn = true;
    lastFanToggle = millis();
  } else if (status.currentTemperature < FAN_ON_TEMPERATURE && isFanOn &&
             millis() - lastFanToggle > FAN_DEBOUNCE_THRESHOLD_MILLIS) {
    logger.info(F("Turning off fan"));
    digitalWrite(FAN_PIN, LOW);
    isFanOn = false;
    lastFanToggle = millis();
  }

  //  logger.debug(F("Reading command"));
  if (receiveCommand()) {
    lastUiHeartbeat = millis();

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(commandJson, receivedChars);

    // Test if parsing succeeds.
    if (error) {
      logger.warn((String("Could not parse command: ") + String(error.c_str()) +
                   String(", '") + String(receivedChars) + String("'"))
                      .c_str());
    } else {
      bool commandPresent = false;

      if (commandJson["target"] != nullptr) {
        commandPresent = true;
        float targetTemperature = commandJson["target"];
        if (targetTemperature != 0 &&
            targetTemperature < MIN_TARGET_TEMPERATURE) {
          enterErrorState(ERROR_TARGET_TEMPERATURE_TOO_LOW);
        } else if (targetTemperature != 0 &&
                   targetTemperature > MAX_TARGET_TEMPERATURE) {
          enterErrorState(ERROR_TARGET_TEMPERATURE_TOO_HIGH);
        } else {
          status.targetTemperature = targetTemperature;
        }
      }

      if (commandJson["log"] != nullptr) {
        commandPresent = true;
        const char *logLevel = commandJson["log"];
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

      if (commandJson["reset"] != nullptr) {
        logger.warn(F("Resetting..."));
        resetFunc();
      }

      if (commandJson["pid_tune"] != nullptr) {
        float kp = commandJson["kp"];
        float ki = commandJson["ki"];
        float kd = commandJson["kd"];
        if (commandJson["pid"] == "top") {
          topHeatingElementPid.SetTunings(kp, ki, kd);
        } else if (commandJson["pid"] == "bottom") {
          bottomHeatingElementPid.SetTunings(kp, ki, kd);
        } else if (commandJson["pid"] == "both") {
          topHeatingElementPid.SetTunings(kp, ki, kd);
          bottomHeatingElementPid.SetTunings(kp, ki, kd);
        } else {
          logger.warn(F("Invalid PID command"));
        }
      }

      if (commandJson["pid_reset"] != nullptr) {
        if (commandJson["reset_pid"] == "top") {
          topHeatingElementPid.Reset();
        } else if (commandJson["reset_pid"] == "bottom") {
          bottomHeatingElementPid.Reset();
        } else if (commandJson["reset_pid"] == "both") {
          topHeatingElementPid.Reset();
          bottomHeatingElementPid.Reset();
        } else {
          logger.warn(F("Invalid PID reset command"));
        }
      }

      if (!commandPresent) {
        logger.warn(F("No command present, received: "));
        serializeJson(commandJson, Serial);
        Serial.println();
      }
    }
    newData = false;
  } else if (lastUiHeartbeat != 0 &&
             millis() - lastUiHeartbeat > UI_STALE_THRESHOLD_MILLIS) {
#ifndef DISABLE_UI_TIMEOUT
    enterErrorState(ERROR_UI_TIMEOUT);
#endif
  }

  // send status
  if (millis() - lastSentStatus > STATUS_INTERVAL_MILLIS) {
    sendStatus();
    lastSentStatus = millis();
  }

  if (status.state == State::FAULT) {
    // make sure heating elements are off
    if (status.topHeatDutyCycle != 0 || status.bottomHeatDutyCycle != 0) {
      logger.warn("Heating elements should be off already! Turning off now...");
      immediateStop();
      delay(50);
    }
    return;
  }

  generateStepResponse();

  if (status.topHeatDutyCycle != lastTopHeatDutyCycle ||
      status.bottomHeatDutyCycle != lastBottomHeatDutyCycle) {
    // set PWM
#ifndef DISABLE_HEATING
    heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.topHeatDutyCycle);
    heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.bottomHeatDutyCycle);
#endif
    lastTopHeatDutyCycle = status.topHeatDutyCycle;
    lastBottomHeatDutyCycle = status.bottomHeatDutyCycle;
  }

  loopTime = micros() - loopTime;
  if (loopTime > LOOP_SLOW_THRESHOLD_MICROS) {
    logger.warn(
        (String("Loop time >5000us: ") + String(loopTime) + String("us"))
            .c_str());
  }
}
