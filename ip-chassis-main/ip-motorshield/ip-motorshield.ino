#include <Arduino.h>

#include <QTRSensors.h>

#include "onboard-led.h"
#include "config.h"
#include "src/motorshield/motorshield.h"
#include "qtr-sensor-util.h"
#include "line-follower.h"
#include "shared-globals.h"
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

  calibrateSensors(qtr, shield.motor(1), shield.motor(2));

#if !ENABLE_WEB_INTERFACE
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

  static uint32_t lastDriveTime = 0;
  static uint32_t driveLoopCounter = 0;
  static uint32_t driveLoopWindowStartMs = 0;
  if (micros() - lastDriveTime >= (1000000 / driveLoopHzTarget))
  {
    lastDriveTime = micros();
    const uint32_t startedAtUs = micros();
    drivingLoop(pid, qtr, shield.motor(1), shield.motor(2));
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
