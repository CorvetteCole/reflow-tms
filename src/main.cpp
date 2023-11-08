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
#include <MAX31865_NonBlocking.h>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_FREQ 1000L

#define PID_ONLY

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 600; // runPid interval = testTimeSec / samples
const uint16_t samples = 1000;
const float inputSpan = 300;
const float outputSpan = 100;
float outputStart = 0;
float outputStep = 30;
float tempLimit = 300;
bool startup = true;

AVR_Slow_PWM heatingElementPwm;

Status status;

// 0.045
float pidCurrentTemperature,
    pidTargetTemperature = 130, pidHeatDutyCycle = 0,
    //                             Kp = 60.0, Ki = 0.0, Kd = 60.0;
//    Kp = 1.3517062446434995, Ki = 0.0031333381510065665, Kd = 98.43382315402697;
    Kp = 100, Ki = 0.025, Kd = 100;

// TODO should have separate PID per stage
// https://github.com/rocketscream/Reflow-Oven-Controller/blob/master/reflowOvenController.ino
// #define TEMPERATURE_ROOM 50
// #define TEMPERATURE_SOAK_MIN 150
// #define TEMPERATURE_SOAK_MAX 200
// #define TEMPERATURE_REFLOW_MAX 250
// #define TEMPERATURE_COOL_MIN 100
// #define SENSOR_SAMPLING_TIME 1000
// #define SOAK_TEMPERATURE_STEP 5
// #define SOAK_MICRO_PERIOD 9000
// #define DEBOUNCE_PERIOD_MIN 50
//
//// ***** PREHEAT STAGE *****
// #define PID_KP_PREHEAT 100
// #define PID_KI_PREHEAT 0.025
// #define PID_KD_PREHEAT 20
//// ***** SOAKING STAGE *****
// #define PID_KP_SOAK 300
// #define PID_KI_SOAK 0.05
// #define PID_KD_SOAK 250
//// ***** REFLOW STAGE *****
// #define PID_KP_REFLOW 300
// #define PID_KI_REFLOW 0.05
// #define PID_KD_REFLOW 350

sTune tuner = sTune(&pidCurrentTemperature, &pidHeatDutyCycle,
                    sTune::CohenCoon_PID, sTune::direct5T, sTune::printOFF);
QuickPID heatingElementPid(&pidCurrentTemperature, &pidHeatDutyCycle,
                           &pidTargetTemperature);

MAX31865 max31865(10);

Logger logger = Logger(LogLevel::DEBUG);

void pwmTimerHandler() { heatingElementPwm.run(); }

void immediateStop() {
  heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                     HEATING_ELEMENT_PWM_FREQUENCY, 0);
  status.heatDutyCycle = 0;
}

void sendStatus() {
  serializeJson(status.toJson(), Serial);
  Serial.println();
}

void soundAlarm() {
  // TODO need to call for cooling
  tone(BUZZER_PIN, 4000);
}

void enterErrorState(uint8_t error) {
  if (status.state != State::FAULT) {
    soundAlarm();
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

  pinMode(FAN_PIN, OUTPUT);
  pinMode(TOP_HEATING_ELEMENT_PIN, OUTPUT);
  pinMode(BOTTOM_HEATING_ELEMENT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

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

  logger.debug(F("Initializing PID..."));

  //  heatingElementPid.SetOutputLimits(0, 100);
  //  heatingElementPid.SetTunings(TOP_HEATING_ELEMENT_KP,
  //  TOP_HEATING_ELEMENT_KI,
  //                               TOP_HEATING_ELEMENT_KD);

  logger.debug(F("Initializing MAX31865_3WIRE..."));

  max31865.begin(MAX31865::RTD_3WIRE, MAX31865::FILTER_60HZ);
  //  max31865.autoConvert(true);

  logger.info(F("Thermal management system started"));

  heatingElementPid.SetOutputLimits(0, 100);
  heatingElementPid.SetSampleTimeUs(PID_INTERVAL_MICROS);

  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec,
                  settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);

  status.targetTemperature = pidTargetTemperature;

//  digitalWrite(FAN_PIN, HIGH);
//  tone(BUZZER_PIN, 4000);
//  status.heatDutyCycle = 100;


#ifdef PID_ONLY
  pidHeatDutyCycle = 100;
  heatingElementPid.SetMode(
      QuickPID::Control::automatic); // the PID is turned on
  heatingElementPid.SetProportionalMode(QuickPID::pMode::pOnErrorMeas);
  heatingElementPid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  heatingElementPid.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
#endif
}

/// Reads the temperature sensor, updating the status struct accordingly.
/// Returns true if the temperature sensor reading was successful, false
/// otherwise.
void readTemperature() {
  if (!max31865.isConversionComplete()) {
    // TODO we should probably have some sort of check to make sure that we
    // catch if the conversion never completes
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
    delay(100); // TODO ?
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
uint32_t lastSentStatus = 0;
// uint32_t lastTemperatureRead = 0;

unsigned long lastPidCompute = 0;
unsigned long loopTime = 0;

void loop() {
  loopTime = micros();
  // send status
  if (millis() - lastSentStatus > STATUS_SEND_INTERVAL) {
    sendStatus();
    lastSentStatus = millis();
  }

  if (status.state == State::FAULT) {
    // if we're in a fault state, don't do anything
    return;
  }

  status.isDoorOpen = !digitalRead(DOOR_PIN);

  readTemperature();
  pidCurrentTemperature = status.currentTemperature;

  if (status.state == State::HEATING && status.isDoorOpen) {
    enterErrorState(ERROR_DOOR_OPENED_DURING_HEATING);
  }

//  return;

#ifdef PID_ONLY
  if (micros() - lastPidCompute > PID_INTERVAL_MICROS + 1000) {
    logger.warn((String("PID interval overrun: ") +
                 String(micros() - lastPidCompute - PID_INTERVAL_MICROS) +
                 String("us"))
                    .c_str());
  }
  heatingElementPid.Compute();
  lastPidCompute = micros();
#else
  switch (tuner.Run()) {
  case sTune::sample: // active once per sample
                      //    tuner.plotter(pidCurrentTemperature,
                      //    pidHeatDutyCycle, pidTargetTemperature,
                      //                  0.5f, 3);
    tuner.printPidTuner(1);
    break;
  case sTune::tunings:
    for (uint8_t i = 0; i < 9; i++) {
      tuner.SetTuningMethod(static_cast<sTune::TuningMethod>(i));
      tuner.printTunings();
    }

    tuner.GetAutoTunings(&Kp, &Ki, &Kd); // active just once when sTune is done
    //        heatingElementPid.SetOutputLimits(0, outputSpan * 0.1);
    heatingElementPid.SetSampleTimeUs(PID_INTERVAL_MICROS);
    pidHeatDutyCycle = outputStep;
    heatingElementPid.SetMode(
        QuickPID::Control::automatic); // the PID is turned on
    heatingElementPid.SetProportionalMode(QuickPID::pMode::pOnMeas);
    heatingElementPid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    heatingElementPid.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
    break;

  case sTune::runPid: // active once per sample period
    heatingElementPid.Compute();
    //    tuner.plotter(pidCurrentTemperature, pidHeatDutyCycle,
    //    pidTargetTemperature,
    //                  0.5f, 3);
    tuner.printPidTuner(1);
    break;
  }
#endif

  if (pidHeatDutyCycle > 100 || pidHeatDutyCycle < 0) {
    logger.warn(F("PID output out of bounds!"));
    Serial.println(pidHeatDutyCycle);
    enterErrorState(ERROR_CURRENT_TEMPERATURE_FAULT);
  }

//  status.heatDutyCycle = static_cast<uint8_t>(pidHeatDutyCycle); // TODO don't do this

  if (status.heatDutyCycle != lastHeatDutyCycle) {
    //    logger.debug(F("Updating PWM"));
    // set PWM
    heatingElementPwm.modifyPWMChannel(0, TOP_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.heatDutyCycle);
    heatingElementPwm.modifyPWMChannel(1, BOTTOM_HEATING_ELEMENT_PIN,
                                       HEATING_ELEMENT_PWM_FREQUENCY,
                                       status.heatDutyCycle);
    lastHeatDutyCycle = status.heatDutyCycle;
  }
  loopTime = micros() - loopTime;
  logger.debug((String("Loop time: ") + String(loopTime) + String("us")).c_str());

  if (loopTime > LOOP_SLOW_THRESHOLD_MICROS) {
    logger.warn(
        (String("Loop time >5000us: ") + String(loopTime) + String("us"))
            .c_str());
  }
}
