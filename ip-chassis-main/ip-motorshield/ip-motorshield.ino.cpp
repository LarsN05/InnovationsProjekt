# 1 "/var/folders/4d/h__q44g92ks7z8brxd1b6h_w0000gn/T/tmpc0kuk0mu"
#include <Arduino.h>
# 1 "/Users/aalohan/InnovationsProjekt/ip-chassis-main/ip-motorshield/ip-motorshield.ino"
#include <Arduino.h>

#include <QTRSensors.h>
#include "onboard-led.h"
#include "config.h"
#include "src/motorshield/motorshield.h"
#include "qtr-sensor-util.h"
#include "line-follower.h"
#include "shared-globals.h"
#include "movement-functions.h"
#include "buzzer.h"
#if ENABLE_WEB_INTERFACE
#include "web-interface.h"
#endif

Motorshield shield;
Mcp23017 &mcp = shield.gpio();
QTRSensors qtr;
PIDController pid(pidControlKp, pidControlKi, pidControlKd, (float)(QTR_LINE_MAX_VALUE / 2));



constexpr float batteryWarn40PercentV = 13.1f;
constexpr float batteryWarn20PercentV = 12.9f;

#if ENABLE_WEB_INTERFACE
void webInterfaceTask(void *);
void maybeBlinkForBatteryLowVoltageWarning();
void setup();
void loop();
#line 27 "/Users/aalohan/InnovationsProjekt/ip-chassis-main/ip-motorshield/ip-motorshield.ino"
void webInterfaceTask(void *)
{
  for (;;)
  {
    TickType_t sleepTicks = pdMS_TO_TICKS(updateWebInterface());
    if (sleepTicks == 0)
    {
      sleepTicks = 1;
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
    playSound(Sound::BatteryLow);
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
    blinkWarningPattern(128, 0, 255, 8);
  }

  initEncodedMotor();
  initBarrierSensor();
  initBuzzer();
  shield.motor(driveMotorLeft).setInverted(invertDriveMotors);
  shield.motor(driveMotorRight).setInverted(invertDriveMotors);

  maybeBlinkForBatteryLowVoltageWarning();
# 117 "/Users/aalohan/InnovationsProjekt/ip-chassis-main/ip-motorshield/ip-motorshield.ino"
  setSwitchAngle(switchCenterDeg + switchOffsetDeg);


#if ENABLE_WEB_INTERFACE
  initWebInterface(pid, &shield);
  xTaskCreatePinnedToCore(webInterfaceTask, "webInterface", 8192, nullptr, 1, nullptr, 0);
#endif






#if 0
  {
    constexpr int testSpeed = 1500;
    constexpr uint32_t testStopMs = 500;
    bool cubeInFront = false;
    uint32_t cubeCount = 0;
    uint32_t lastPrintMs = 0;
    uint16_t windowMin = UINT16_MAX;
    uint16_t windowMax = 0;

    Serial.printf("Revolver test: speed %d, barrier threshold %u us\n", testSpeed, barrierThreshold);
#if !ENABLE_WEB_INTERFACE
    robotMotionEnabled = true;
#endif
    bool turning = false;
    for (;;)
    {
      const uint16_t value = readBarrierSensor();
      const bool detected = value < barrierThreshold;


      if (robotMotionEnabled != turning)
      {
        turning = robotMotionEnabled;
        if (turning)
          shield.motor(revolverMotorNumber).setSpeed(testSpeed);
        else
          shield.motor(revolverMotorNumber).brake();
        Serial.printf("Revolver test: %s\n", turning ? "started" : "stopped");
      }

      if (turning && detected && !cubeInFront)
      {

        cubeInFront = true;
        ++cubeCount;
        shield.motor(revolverMotorNumber).brake();
        Serial.printf("Cube %lu detected (%u us), stopping %lu ms\n", (unsigned long)cubeCount, value, (unsigned long)testStopMs);
        encodedMotorDelay(testStopMs);
        if (robotMotionEnabled)
          shield.motor(revolverMotorNumber).setSpeed(testSpeed);
      }
      else if (!detected)
      {
        cubeInFront = false;
      }



      if (value < windowMin) windowMin = value;
      if (value > windowMax) windowMax = value;
      if (millis() - lastPrintMs >= 100)
      {
        lastPrintMs = millis();
        Serial.printf("Barrier sensor: now %4u us   min %4u   max %4u   threshold %u %s\n",
                      value, windowMin, windowMax, barrierThreshold, detected ? "<- CUBE" : "");
        windowMin = UINT16_MAX;
        windowMax = 0;
      }
      encodedMotorDelay(5);
    }
  }
#endif



  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSOR_PINS, QTR_SENSOR_COUNT);
  qtr.setTimeout(1000);
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
  setOnboardLedColor(0, 200, 200);
  delay(3000);
  robotMotionEnabled = true;
  setHeartbeatStandaloneActive();
#endif

  clearOnboardLed();
  playSound(Sound::Startup);
}

void loop()
{
  blinkPeriodically();
  updateEncodedMotor();


  static uint32_t lastBarrierReadMs = 0;
  if (millis() - lastBarrierReadMs >= 100)
  {
    lastBarrierReadMs = millis();
    readBarrierSensor();
  }


  if (testShotRequested && !robotMotionEnabled)
  {
    testShotRequested = false;
    testShotActive = true;
    const bool shotOk = shootSequence(testShotRight, testShotRpm);
    const bool cancelled = !testShotActive;
    testShotActive = false;
    adjustSpeed(0);
    Serial.printf("Test shot: %s\n", cancelled ? "cancelled" : (shotOk ? "done" : "failed (rpm too low or no package)"));
  }

#if ENCODER_MOTOR_TEST_MODE

  printEncodedMotorDebug();
# 262 "/Users/aalohan/InnovationsProjekt/ip-chassis-main/ip-motorshield/ip-motorshield.ino"
  return;
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