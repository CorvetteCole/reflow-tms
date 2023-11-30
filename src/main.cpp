#include <Arduino.h>
#include <ArduinoJson.h>
#define MAXSIZE 20
#include <MPC.h>
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

char receivedChars[INPUT_BUFFER_SIZE];
bool newData = false; // represents whether the receivedChars array is ready to
                      // be parsed

MAX31865 max31865(10);

Logger logger = Logger(LogLevel::INFO);

MatDataType_t L_phi = 3.491353;
MatDataType_t e_V = 0.001;
MatDataType_t e_g = 0.001;
uint32_t max_iter = 1000;
uint32_t N = 100;

MatDataType_t A_arr[4] = {0.999145, -1e-06, 0.099957, 1.0};
MatDataType_t B_arr[2] = {0.099957, 0.004999};
MatDataType_t Q_arr[4] = {1.0, 0.0, 0.0, 1.0};
MatDataType_t R_arr[1] = {1.0};
MatDataType_t QN_arr[4] = {17.771051, 10.012292, 10.012292, 17.835026};
MatDataType_t F_arr[8] = {1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
MatDataType_t G_arr[4] = {0.0, 0.0, 1.0, -1.0};
MatDataType_t c_arr[4] = {270, 20, 100, 0};
MatDataType_t FN_arr[4] = {-1.627747, -0.917459, 0.006926, 0.003858};
MatDataType_t cN_arr[2] = {0.0, 0.0};

Matrix A = Matrix(2, 2, A_arr);
Matrix B = Matrix(2, 1, B_arr);
Matrix Q = Matrix(2, 2, Q_arr);
Matrix R = Matrix(1, 1, R_arr);
Matrix QN = Matrix(2, 2, QN_arr);
Matrix F = Matrix(4, 2, F_arr);
Matrix G = Matrix(4, 1, G_arr);
Matrix c = Matrix(4, 1, c_arr);
Matrix FN = Matrix(2, 2, FN_arr);
Matrix cN = Matrix(2, 1, cN_arr);

MPCController mpc = MPCController(L_phi, e_V, e_g, max_iter, N, A, B, Q, R, QN,
                                  F, G, c, FN, cN);

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
  status.print();
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

  if (!ITimer1.attachInterrupt(HW_TIMER_INTERVAL_FREQ, pwmTimerHandler)) {
    logger.error(
        F("Can't start ITimer1"));
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

  logger.debug(F("Initializing MPC..."));

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865::RTD_3WIRE, MAX31865::FILTER_60HZ);

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

void computeMPC() {
  static unsigned long lastCompute = 0;
  static Matrix state = Matrix(1, 1);
  static Matrix control = Matrix(1, 1);

  if (status.state != State::HEATING) {
    // only compute PID if we are heating, since it is expensive
    return;
  }

  if (lastCompute != 0) {
    // if we are past the control interval, we should log a warning
    if (micros() - lastCompute >
        MPC_INTERVAL_MICROS + 1000) { // TODO do we need this +1000?
      logger.warn(F("Control loop interval too long!"));
    } else if (micros() - lastCompute < MPC_INTERVAL_MICROS - 1000) {
      // we need to compute at exactly the control interval. So just return if
      // we are not there yet. If we are within 1ms of the control interval, we
      // will compute anyway.
      return;
    }
  }

  state(0, 0) = status.currentTemperature;
  control = mpc(state);
  lastCompute = micros();

  float heatDutyCycle = control(0, 0);

  // clamp heat duty cycle to 0-100
  if (heatDutyCycle > 100) {
    logger.warn(F("MPC heat duty cycle > 100%!"));
    heatDutyCycle = 100;
  } else if (heatDutyCycle < 0) {
    logger.warn(F("MPC heat duty cycle < 0%!"));
    heatDutyCycle = 0;
  }

  // we can statically cast to uint8_t because the output limits are set
  // to 0-100
  status.topHeatDutyCycle = static_cast<uint8_t>(heatDutyCycle);
  status.bottomHeatDutyCycle = static_cast<uint8_t>(heatDutyCycle);
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

  if (status.targetTemperature == 0) {
    if (status.state != State::IDLE) {
      logger.debug(F("Target temperature is 0, disabling heating elements"));
      status.state = State::IDLE;
      status.topHeatDutyCycle = 0;
      status.bottomHeatDutyCycle = 0;
    }
  } else {
    if (status.state != State::COOLING &&
        status.currentTemperature > status.targetTemperature &&
        status.currentTemperature - status.targetTemperature > 20) {
      logger.info(F("Started cooling"));
      status.topHeatDutyCycle = 0;
      status.bottomHeatDutyCycle = 0;
      status.state = State::COOLING;
    } else if (status.state != State::HEATING &&
               status.targetTemperature > status.currentTemperature) {
      logger.info(F("Started heating"));
      status.state = State::HEATING;
    }
  }

  if (status.state == State::HEATING || status.state == State::IDLE) {
    // only compute when we need it - we're heating
    computeMPC();
  } else if (status.state == State::COOLING) {
    if (status.isDoorOpen) {
      noTone(BUZZER_PIN);
    } else {
#ifndef DISABLE_BUZZER
      tone(BUZZER_PIN, ATTENTION_FREQUENCY);
#endif
    }
  }

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
