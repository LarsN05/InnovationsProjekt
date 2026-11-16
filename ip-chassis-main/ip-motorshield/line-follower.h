
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
    uint32_t lastUpdateUs; // 0 = no previous step (after construction or reset())

    PIDController(float kp, float ki, float kd, float target)
        : kp(kp), ki(ki), kd(kd), target(target), integral(0.0f), prevError(0.0f), lastUpdateUs(0) {}

    // Clears the controller memory. Call before resuming line following after any manoeuvre.
    void reset()
    {
        integral = 0.0f;
        prevError = 0.0f;
        lastUpdateUs = 0;
    }
};

// One PID step. dt is measured from the previous call (falls back to 1 / driveLoopHzTarget after reset()).
float calculatePIDstep(PIDController &pid, float measurement);
void updateLineStatus();
void setMotorSpeeds(int signal, Motor &motorL, Motor &motorR);
void drivingLoop(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR);
