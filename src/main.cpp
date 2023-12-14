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

#include <MAX31865_NonBlocking.h>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_FREQ 1000L

AVR_Slow_PWM heatingElementPwm;

Status status;

uint8_t receiveBuffer[INPUT_BUFFER_SIZE];
uint8_t receiveBufferSize = 0;

bool newData = false; // represents whether the receivedChars array is ready to
                      // be parsed

MAX31865 max31865(10);

Logger logger = Logger(LogLevel::INFO);

void pwmTimerHandler() { heatingElementPwm.run(); }

void stopHeating() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
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
    stopHeating();
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
        receiveBuffer[ndx] = rc;
        ndx++;
        if (ndx >= INPUT_BUFFER_SIZE) {
          ndx = INPUT_BUFFER_SIZE - 1;
        }
      } else {
        receiveBuffer[ndx] = endMarker;
        receiveBufferSize = ndx + 1;
        receiveInProgress = false;
        ndx = 0;
        return true;
      }
    } else if (rc == startMarker) {
      receiveBuffer[ndx] = startMarker;
      ndx++;
      receiveInProgress = true;
    }
  }
  return false;
}

void handleCommand() {
  static StaticJsonDocument<32> commandJson;

  // Deserialize the JSON document
  DeserializationError error =
      deserializeJson(commandJson, receiveBuffer, receiveBufferSize);

  // Test if parsing succeeds.
  if (error) {
    logger.warn(F("deserializeJson() failed"));
    return;
  }

  bool commandPresent = false;

  if (commandJson["state"] != nullptr) {
    commandPresent = true;
    int state = commandJson["state"];
    if (state < 0 || state > 2) {
      logger.warn(F("Invalid state"));
    } else {
      status.state = static_cast<State>(state);
    }
  }

  if (commandJson["pwm"] != nullptr) {
    commandPresent = true;
    uint8_t heatDutyCycle = commandJson["pwm"];
    if (heatDutyCycle > 100) {
      logger.warn(F("Heat duty cycle must be between 0 and 100"));
    } else {
      status.heatDutyCycle = heatDutyCycle;
    }
  }

  if (commandJson["log"] != nullptr) {
    commandPresent = true;
    int logLevel = commandJson["log"];
    if (0 < logLevel && logLevel < 3) {
      logger.logLevel = static_cast<LogLevel>(logLevel);
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

  newData = false;
}

// TODO: detect current temperature not rising during heating
// TODO: implement CRC16 checksums for commands
void loop() {
  static uint8_t lastHeatDutyCycle = 0;
  static unsigned long lastUiHeartbeat = 0;
  static unsigned long lastSentStatus = 0;
  static unsigned long loopTime = 0;
  static unsigned long lastFanToggle = 0;
  static bool isFanOn = false;

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
    handleCommand();
    lastUiHeartbeat = millis();
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

  switch (status.state) {
  case State::HEATING:
    if (status.heatDutyCycle != lastHeatDutyCycle) {
      // set PWM
#ifndef DISABLE_HEATING
      heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                         HEATING_ELEMENT_PWM_FREQUENCY,
                                         status.heatDutyCycle);
      heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                         HEATING_ELEMENT_PWM_FREQUENCY,
                                         status.heatDutyCycle);
#endif
      lastHeatDutyCycle = status.heatDutyCycle;
    }
    break;
  case State::COOLING:
    if (!status.isDoorOpen) {
#ifndef DISABLE_BUZZER
      tone(BUZZER_PIN, ATTENTION_FREQUENCY);
#endif
    }
  case State::IDLE:
    if (status.isDoorOpen) {
      noTone(BUZZER_PIN);
    }
  case State::FAULT:
    // make sure heating elements are off
    stopHeating();
    break;
  }

  loopTime = micros() - loopTime;
  if (loopTime > LOOP_SLOW_THRESHOLD_MICROS) {
    logger.warn(
        (String("Loop time >5000us: ") + String(loopTime) + String("us"))
            .c_str());
  }
}
