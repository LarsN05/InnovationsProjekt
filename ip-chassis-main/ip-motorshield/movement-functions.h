#pragma once

#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"

// Mission helpers: every waiting step of a checkpoint action goes through missionWait(),
// so the web interface Stop button takes effect within a few milliseconds.
bool missionWait(uint32_t ms);                        // like encodedMotorDelay(), but returns false (and calls abortMission()) once Stop was pressed
bool missionAborted();                                // true once Stop was pressed (robotMotionEnabled == false)
void abortMission();                                  // brakes the drive motors and the revolver, stops the shooter
void updateBatteryCompensation();                     // measures the battery once; call at the start of a manoeuvre
int compensateForBattery(int speed);                  // scales a speed by nominal / measured battery voltage

bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR);
// Turns on the spot in short pulses until the line is under the middle sensor. true = centered.
bool centerOnLine(QTRSensors &qtr, Motor &motorL, Motor &motorR);
// Turns on the spot in short pulses until the line position is targetOffset away from the middle of the sensor bar
// (> 0 = turn right, the line moves to the left side; < 0 = turn left). true = reached, false = timeout/Stop.
bool turnToLineOffset(QTRSensors &qtr, Motor &motorL, Motor &motorR, int targetOffset, uint32_t timeoutMs);
// Drives (speed < 0 = backwards) until both outer sensors see the cross line; each wheel stops as soon as
// the sensor on its side is on the line, so the robot ends square to the cross line. true = square.
bool squareOnCrossLine(QTRSensors &qtr, Motor &motorL, Motor &motorR, int speed, uint32_t timeoutMs);
bool allignOnCheckpoint(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR);
void initEncodedMotor();                              // call once in setup()
void encodedMotorSetSpeed(int speed);                 // open loop: -100..100 (%)
void adjustSpeed(int rpm);                            // closed loop: target rpm, negative = reverse, 0 = stop
void updateEncodedMotor();                            // call as often as possible in loop()
void encodedMotorDelay(uint32_t ms);                  // like delay(), but keeps regulating the encoder motor
void runEncodedMotor(int rpm, uint32_t durationMs);   // blocking: regulate to rpm for durationMs, then stop
float encodedMotorRpm();                              // measured speed in rpm
float encodedMotorTargetRpmValue();                   // current target rpm
long encodedMotorTicks();                             // encoder position in ticks
void printEncodedMotorDebug();                        // call in loop(): prints the controller state every 200 ms
void initBarrierSensor();                             // call once in setup()
uint16_t readBarrierSensor();                         // raw value in us, blocks up to barrierTimeoutUs
bool barrierDetected();                               // true if the value is below barrierThreshold
bool turnUntilPackage();                              // true if a package was detected before the timeout
bool shootSequence(bool switchRight, int rpm, bool stopShooterAfter = true); // sets the switch (true = right slide, if enableSlideSwitch), spins up, shoots one cube; true if it worked
void setSwitch(bool left);                            // moves the switch servo to the left or right slide (only if enableSlideSwitch)
float setSwitchAngle(float deg);                      // moves the switch servo to an angle (always), returns the clamped angle