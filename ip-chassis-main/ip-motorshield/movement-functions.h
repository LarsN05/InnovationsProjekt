#pragma once

#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"

bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR);
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