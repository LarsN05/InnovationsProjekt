
#pragma once
#include "src/motorshield/motor.h"
#include <QTRSensors.h>

enum LineStatus
{
  NORMAL, // Black line is in the middle or slightly to the left or right
  STOP,  // All sensors detect black line, e.g. at a checkpoint
  STATION  // Outer sensors detect line but not the inner ones
};

class PIDController
{
public:
    float kp;
    float ki;
    float kd;
    float target;
    float integral;
    float prevError;

    PIDController(float kp, float ki, float kd, float target)
        : kp(kp), ki(ki), kd(kd), target(target), integral(0.0f), prevError(0.0f) {}
};

float calculatePIDstep(PIDController &pid, float measurement, float executionFrequency);
void updateLineStatus();
void setMotorSpeeds(int signal, Motor &motorL, Motor &motorR);
void drivingLoop(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR);
