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

// Drive motor ports (driveMotorLeft / driveMotorRight / invertDriveMotors) are set in config.h.

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
    blinkWarningPattern(128, 0, 255, 8);  // violet: 8 blinks for motor shield init failure
  }

  initEncodedMotor();
  initBarrierSensor();
  initBuzzer(); // after shield.begin(): the drive motor PWM must already own its LEDC timer
  shield.motor(driveMotorLeft).setInverted(invertDriveMotors);
  shield.motor(driveMotorRight).setInverted(invertDriveMotors);

  maybeBlinkForBatteryLowVoltageWarning();

  /*for(int i = 0; i < 400; i++){
    shield.motor(5).setSpeed(-800);    // kurzer Schub zum Losbrechen
    encodedMotorDelay(50);
    shield.motor(5).setSpeed(0);
    encodedMotorDelay(500);
  }*/
  /*shield.servo(1).setAngleRange(135);
  shield.servo(1).setAngle(130);
  delay(4000);
  shield.servo(1).setAngle(135);
  delay(4000);
  shield.servo(1).setAngle(125);
  delay(200000);
  */
  /*turnUntilPackage();
  delay(20000);*/
  /*turnUntilPackage();
  delay(20000);*/

  //code to directly test:
  // minimum = 144, 140 
  // maximum = 175, 185
  //setSwitchAngle(140);

  // The web interface starts here, before the test below, so it also works while the test runs.
#if ENABLE_WEB_INTERFACE
  initWebInterface(pid, &shield);
  xTaskCreatePinnedToCore(webInterfaceTask, "webInterface", 8192, nullptr, 1, nullptr, 0);
#endif

  // Revolver + barrier sensor test: the revolver turns, stops for 0.5 s whenever a cube reaches the
  // barrier sensor, then keeps turning. Runs forever: calibration and line following never start.
  // With the web interface on, the revolver only turns after "Start Moving" and stops on "Stop Moving";
  // the barrier value is also shown in the web interface telemetry.
  // Set to 0 to disable.
#if 1
  {
    constexpr int testSpeed = 1500;            // revolver PWM (-4095..4095); set to 0 to only read the sensor (hold a cube in front by hand)
    constexpr uint32_t testStopMs = 500;       // stop time per cube
    bool cubeInFront = false;                  // true while the current cube is still in front of the sensor
    uint32_t cubeCount = 0;
    uint32_t lastPrintMs = 0;
    uint16_t windowMin = UINT16_MAX;
    uint16_t windowMax = 0;

    Serial.printf("Revolver test: speed %d, barrier threshold %u us\n", testSpeed, barrierThreshold);
#if !ENABLE_WEB_INTERFACE
    robotMotionEnabled = true; // no web interface: start right away
#endif
    bool turning = false;
    for (;;)
    {
      const uint16_t value = readBarrierSensor();
      const bool detected = value < barrierThreshold;

      // Start / Stop from the web interface
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
        // New cube: stop, wait, continue
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
        cubeInFront = false; // sensor is clear again: the next detection is a new cube
      }

      // Sensor values for finding barrierThreshold: every 100 ms the lowest / highest reading of that window,
      // so a cube passing quickly is not missed. Small value = something in front of the sensor.
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
  playSound(Sound::Startup);
}

void loop()
{
  blinkPeriodically();
  updateEncodedMotor();

  // Barrier sensor reading for the web interface telemetry (helps to choose barrierThreshold).
  static uint32_t lastBarrierReadMs = 0;
  if (millis() - lastBarrierReadMs >= 100)
  {
    lastBarrierReadMs = millis();
    readBarrierSensor();
  }

#if ENCODER_MOTOR_TEST_MODE
  
  printEncodedMotorDebug(); // controller state every 200 ms on Serial

  // Raw QTR values every 500 ms: move a black line under the sensors, the values must rise clearly
  // (temporarily disabled to keep the Serial Monitor readable while tuning the encoder motor)
  /*static uint32_t lastQtrPrintMs = 0;
  if (millis() - lastQtrPrintMs >= 500)
  {
    lastQtrPrintMs = millis();
    printSensorValues(qtr);
    Serial.printf("Barrier sensor: %u us\n", readBarrierSensor());
  }*/
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
