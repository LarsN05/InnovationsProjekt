#include "line-follower.h"
#include "config.h"
#include "shared-globals.h"
#include <Arduino.h>
#include "movement-functions.h"
#include "mission.h"
#include "onboard-led.h"
#include "buzzer.h"

float calculatePIDstep(PIDController &pid, float measurement)
{
    pidError = (int)(pid.target - measurement);

    // dt from the time since the previous step; after reset() fall back to the nominal loop period.
    const uint32_t nowUs = micros();
    const bool firstStep = (pid.lastUpdateUs == 0);
    float dt = 1.0f / (float)max(driveLoopHzTarget, 1); // normally 40 Hz, so dt = 0.025s
    if (!firstStep)
    {
        const float measuredDt = (nowUs - pid.lastUpdateUs) * 1e-6f;
        if (measuredDt > 0.0f && measuredDt < 0.2f)
        {
            dt = measuredDt;
        }
    }
    pid.lastUpdateUs = nowUs;
    if (firstStep)
    {
        pid.prevError = pidError; // no derivative kick on the first step after a reset
    }

    // integral windup prevention
    const float maximumOutput = PWM_MAX_VALUE;
    float integralLimit = maximumOutput / pid.ki;
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
    int position = qtr.readLineBlack(qtrSensorValues);
    updateLineStatus();

    // Checkpoint detection: the cross line must be seen on checkpointConfirmReads consecutive reads.
    // The counter keeps running while the robot is stopped, so a cross line the robot is
    // standing on (or is carried over) when Start is pressed is not counted.
    static uint8_t crossLineReads = 0;
    if (lineStatus == STOP)
    {
        if (crossLineReads < 255)
        {
            ++crossLineReads;
        }
    }
    else
    {
        crossLineReads = 0;
    }
    const bool crossLineConfirmed = (crossLineReads == checkpointConfirmReads);

    if (!robotMotionEnabled)
    {
        controlSignal = 0.0f;
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        if (brakeWhenStopped)
        {
            motorL.brake();
            motorR.brake();
        }
        else
        {
            motorL.stop();
            motorR.stop();
        }
        pid.reset(); // start from a clean controller when Start is pressed again
        oldLineStatus = lineStatus;
        return;
    }

    controlSignal = calculatePIDstep(pid, position);
    setMotorSpeeds(controlSignal, motorL, motorR);

    // The lockout starts when the checkpoint action has finished (not when the cross line was seen),
    // so a cross line the robot is still on or drives over right after a manoeuvre (e.g. the 180 turn) is not counted again.
    static uint32_t checkpointLockoutStartMs = 0;
    const bool lockoutOver = (millis() - checkpointLockoutStartMs) >= checkpointLockoutMs;
    if (crossLineConfirmed && lockoutOver)
    {
        checkpointCounter++;
        lastCheckpointTimeMs = millis();

        MissionContext ctx{pid, qtr, motorL, motorR};
        const bool ok = runCheckpointAction(checkpointCounter, ctx);
        checkpointLockoutStartMs = millis();

        if (!robotMotionEnabled)
        {
            abortMission(); // Stop pressed during the action, or the run finished
        }
        else if (!ok)
        {
            abortMission();
            robotMotionEnabled = false;
            Serial.printf("Checkpoint %d: action failed, robot stopped\n", checkpointCounter);
            if (enableMissionSounds)
            {
                playSound(Sound::Error);
            }
            blinkWarningPattern(255, 0, 0, 10);
        }
    }
    oldLineStatus = lineStatus;
}
