#include "line-follower.h"
#include "config.h"
#include "shared-globals.h"
#include <Arduino.h>
#include "robot-control.h"

float calculatePIDstep(PIDController &pid, float measurement, float executionFrequency)
{
    pidError = (int)(pid.target - measurement);

    // integral windup prevention
    const float maximumOutput = PWM_MAX_VALUE;
    float integralLimit = maximumOutput / pid.ki;
    float dt = 1.0f / executionFrequency; // normally 40 Hz, so dt = 0.025s
    pid.integral += pidError * dt;
    pid.integral = constrain(pid.integral, -integralLimit, integralLimit);

    // integral reset to prevent overshooting after long build-up when approaching the line from far away
    bool isCrossingLine = (pidError > 0) != (pid.prevError > 0);
    if (isCrossingLine)
    {
        pid.integral = 0;
    }

    // Derivative term (indiferent if based on pidError or measurement, since target is constant (=0))
    // (pidError - previousError) / dt;
    // == (-measurement - (-prevMeasurement)) / dt;  if target is constant
    // == -(measurement - prevMeasurement) / dt;
    float derivative = (pidError - pid.prevError) / dt;

    float control = pid.kp * pidError + pid.ki * pid.integral + pid.kd * derivative;

    pid.prevError = pidError;

    return control;
}

void updateLineStatus()
{
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
    {
        if (qtrSensorValues[i] > BLACKLINE_THRESHOLD)
        {
            qtrSensorStatus[i] = 1;
        }
        else
        {
            qtrSensorStatus[i] = 0;
        }
    }

    if (qtrSensorStatus[0] == 1 && qtrSensorStatus[4] == 1 && (qtrSensorStatus[1] == 0 || qtrSensorStatus[2] == 0 || qtrSensorStatus[3] == 0))
    {
        lineStatus = STATION;
    }
    else if (qtrSensorStatus[0] == 1 && qtrSensorStatus[1] == 1 && qtrSensorStatus[2] == 1 && qtrSensorStatus[3] == 1 && qtrSensorStatus[4] == 1)
    {
        lineStatus = STOP;
    }
    else
    {
        lineStatus = NORMAL;
    }
}

/*
 *  @summary overcome static friction in low speed range
 */
static int compensateDeadzoneWithFlip(int speedCommand)
{
    if (speedCommand == 0)
    {
        return 0;
    }

    int sign = (speedCommand > 0) ? 1 : -1;
    int magnitude = abs(speedCommand);

    if (magnitude >= minPWMtoMoveMotor)
    {
        // Keep direction; ensure we overcome stiction
        int pwm = magnitude; // already ≥ MIN
        if (pwm > PWM_MAX_VALUE)
        {
            pwm = PWM_MAX_VALUE;
        }
        return sign * pwm;
    }
    else
    {
        // Flip direction and push magnitude above MIN
        int flippedMag = 2 * minPWMtoMoveMotor - magnitude; // e.g., magnitude=44, MIN=45 -> -46
        if (flippedMag > PWM_MAX_VALUE)
        {
            flippedMag = PWM_MAX_VALUE;
        }
        return -sign * flippedMag;
    }
}

void setMotorSpeeds(int signal, Motor &motorL, Motor &motorR)
{
    if (hardwareSettings.invertSteering)
    {
        signal = -signal;
    }

    leftMotorSpeed = constrain(defaultMotorSpeed + signal, -PWM_MAX_VALUE, PWM_MAX_VALUE);
    rightMotorSpeed = constrain(defaultMotorSpeed - signal, -PWM_MAX_VALUE, PWM_MAX_VALUE);

    if (enableMotorDeadzoneCompensation)
    {
        leftMotorSpeed = compensateDeadzoneWithFlip(leftMotorSpeed);
        rightMotorSpeed = compensateDeadzoneWithFlip(rightMotorSpeed);
    }

    motorL.setSpeed(leftMotorSpeed);
    motorR.setSpeed(rightMotorSpeed);
}

void drivingLoop(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR)
{
    linePosition = qtr.readLineBlack(qtrSensorValues);
    updateLineStatus();

    if (robotMode == RobotMode::Manual)
    {
        // Motors are driven from the web interface; only keep the sensor readings fresh.
        controlSignal = 0.0f;
        oldLineStatus = lineStatus;
        return;
    }

    if (!robotMotionEnabled)
    {
        controlSignal = 0.0f;
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        if (brakeWhenStopped || robotMode == RobotMode::Paused)
        {
            motorL.brake();
            motorR.brake();
        }
        else
        {
            motorL.stop();
            motorR.stop();
        }
        oldLineStatus = lineStatus;
        return;
    }

    controlSignal = calculatePIDstep(pid, linePosition, driveLoopHzTarget);
    setMotorSpeeds(controlSignal, motorL, motorR);

    if (lineStatus != oldLineStatus)
    {
        if (lineStatus == STOP)
        {
            checkpointCounter++;
            lastCheckpointTimeMs = millis();
            // What happens at each checkpoint is defined by the mission (see robot-control.cpp),
            // which can be edited live from the web interface.
            handleCheckpoint(pid, qtr, motorL, motorR);
        }
        oldLineStatus = lineStatus;
    }
}
