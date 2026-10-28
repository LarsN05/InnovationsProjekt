#include <Arduino.h>

#include <QTRSensors.h>

#include "onboard-led.h"
#include "config.h"
#include "src/motorshield/motorshield.h"
#include "qtr-sensor-util.h"
#include "line-follower.h"
#include "shared-globals.h"
#include "movement-functions.h"
#if ENABLE_WEB_INTERFACE
#include "web-interface.h"
#endif

Motorshield shield;
Mcp23017 &mcp = shield.gpio();
QTRSensors qtr;
PIDController pid(pidControlKp, pidControlKi, pidControlKd, (float)(QTR_LINE_MAX_VALUE / 2));

// Drive motor ports: swap these two numbers if the robot steers away from the line
constexpr uint8_t driveMotorLeft = 1;
constexpr uint8_t driveMotorRight = 2;
// Set to true if the robot drives backwards (sensor bar trailing) instead of forwards
constexpr bool invertDriveMotors = false;

constexpr float batteryWarn40PercentV = 13.1f;
constexpr float batteryWarn20PercentV = 12.9f;

#if ENABLE_WEB_INTERFACE
void webInterfaceTask(void *)
{
  for (;;)
  {
    TickType_t sleepTicks = pdMS_TO_TICKS(updateWebInterface());
    if (sleepTicks == 0)
    {
      sleepTicks = 1; // vTaskDelay(0) only yields; the core 0 idle task must run to feed the watchdog.
    }
    vTaskDelay(sleepTicks);
  }
}
#endif

void maybeBlinkForBatteryLowVoltageWarning()
{
  const float batteryVoltageV = shield.batteryVoltage();
  Serial.printf("Battery voltage: %.2f V\n", batteryVoltageV);

  if (batteryVoltageV < 0.1f)
  {
    Serial.println("Battery warning: Likely no external battery connected or power switch off.");
    blinkWarningPattern(128, 0, 0, 3);
  }
  else if (batteryVoltageV < batteryWarn20PercentV)
  {
    Serial.println("Battery warning: below 20%, blinking red 5x.");
    blinkWarningPattern(128, 0, 0, 5);
  }
  else if (batteryVoltageV < batteryWarn40PercentV)
  {
    Serial.println("Battery warning: below 40%, blinking orange 5x.");
    blinkWarningPattern(128, 50, 0, 5);
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Motorshield firmware starting up...");
  initOnboardLed();
  setOnboardLedColor(255, 0, 0);

  if (!shield.begin())
  {
    Serial.println("Motorshield warning: Initialization failed. Maybe not all I2C chips responded.");
    blinkWarningPattern(128, 0, 255, 8);  // violet: 8 blinks for motor shield init failure
  }

  initEncodedMotor();
  shield.motor(driveMotorLeft).setInverted(invertDriveMotors);
  shield.motor(driveMotorRight).setInverted(invertDriveMotors);

  maybeBlinkForBatteryLowVoltageWarning();
  


#if ENABLE_WEB_INTERFACE
  initWebInterface(pid, &shield);
  xTaskCreatePinnedToCore(webInterfaceTask, "webInterface", 8192, nullptr, 1, nullptr, 0);
#endif

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSOR_PINS, QTR_SENSOR_COUNT);
  qtr.setTimeout(1000); // Reduced from default 2500 to increase control-loop frequency.
  printSensorValues(qtr);

  delay(200);
  setOnboardLedColor(0, 255, 0);
  delay(200);
  setOnboardLedColor(0, 0, 255);
  delay(200);

#if ENCODER_MOTOR_TEST_MODE
  Serial.println("Encoder motor test mode: no calibration, no line following.");
#else
  calibrateSensors(qtr, shield.motor(driveMotorLeft), shield.motor(driveMotorRight));
#endif

#if !ENABLE_WEB_INTERFACE && !ENCODER_MOTOR_TEST_MODE
  setOnboardLedColor(0, 200, 200); // solid cyan: auto-start countdown
  delay(3000);
  robotMotionEnabled = true;
  setHeartbeatStandaloneActive();
#endif

  clearOnboardLed();
}

void loop()
{
  blinkPeriodically();
  updateEncodedMotor();

#if ENCODER_MOTOR_TEST_MODE
  
  printEncodedMotorDebug(); // controller state every 200 ms on Serial

  // Raw QTR values every 500 ms: move a black line under the sensors, the values must rise clearly
  static uint32_t lastQtrPrintMs = 0;
  if (millis() - lastQtrPrintMs >= 500)
  {
    lastQtrPrintMs = millis();
    printSensorValues(qtr);
  }
  return; // drive motors stay off, only the encoder motor runs
#endif

  static uint32_t lastDriveTime = 0;
  static uint32_t driveLoopCounter = 0;
  static uint32_t driveLoopWindowStartMs = 0;
  if (micros() - lastDriveTime >= (1000000 / driveLoopHzTarget))
  {
    lastDriveTime = micros();
    const uint32_t startedAtUs = micros();
    drivingLoop(pid, qtr, shield.motor(driveMotorLeft), shield.motor(driveMotorRight));
    driveLoopExecUsMeasured = static_cast<float>(micros() - startedAtUs);

    ++driveLoopCounter;
    const uint32_t nowMs = millis();
    if (driveLoopWindowStartMs == 0)
    {
      driveLoopWindowStartMs = nowMs;
    }
    if (nowMs - driveLoopWindowStartMs >= 1000)
    {
      driveLoopHzMeasured = (1000.0f * static_cast<float>(driveLoopCounter)) / static_cast<float>(nowMs - driveLoopWindowStartMs);
      driveLoopCounter = 0;
      driveLoopWindowStartMs = nowMs;
    }
  }
}
