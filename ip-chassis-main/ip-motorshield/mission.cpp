#include "mission.h"

#include "config.h"
#include "movement-functions.h"
#include "shared-globals.h"
#include "src/motorshield/motorshield.h"

#include <Arduino.h>

extern Motorshield shield;

namespace
{

// ---------------------------------------------------------------------------
// Drive helpers
// ---------------------------------------------------------------------------

void brakeDrive(MissionContext &ctx)
{
    ctx.motorL.brake();
    ctx.motorR.brake();
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
}

// One control step of a straight move: ramps the speed up, scales it for the battery voltage
// and, when steering, keeps the robot on the line with the PID (forward only, the sensor bar is at the front).
void driveStep(MissionContext &ctx, int speed, uint32_t moveStartMs, bool steer)
{
    constexpr float rampStartFraction = 0.7f; // start high enough to overcome static friction
    const uint32_t elapsedMs = millis() - moveStartMs;
    const float ramp = (elapsedMs >= moveRampMs)
                           ? 1.0f
                           : rampStartFraction + (1.0f - rampStartFraction) * (float)elapsedMs / (float)moveRampMs;
    const int base = compensateForBattery((int)(speed * ramp));

    const int position = ctx.qtr.readLineBlack(qtrSensorValues);
    updateLineStatus();

    int correction = 0;
    if (steer && speed > 0)
    {
        correction = (int)calculatePIDstep(ctx.pid, position);
    }

    leftMotorSpeed = constrain(base + correction, -PWM_MAX_VALUE, PWM_MAX_VALUE);
    rightMotorSpeed = constrain(base - correction, -PWM_MAX_VALUE, PWM_MAX_VALUE);
    ctx.motorL.setSpeed(leftMotorSpeed);
    ctx.motorR.setSpeed(rightMotorSpeed);
}

enum class CrossLine
{
    Reach, // drive until all sensors see black
    Leave  // drive until the sensors no longer all see black
};

// Drives straight (negative speed = backwards) until the checkpoint cross line is reached or left,
// then keeps going for extraMs and brakes. Returns false on Stop or if the cross line was not found in time.
bool driveFromCrossLine(MissionContext &ctx, int speed, CrossLine edge, uint32_t extraMs)
{
    const bool steer = speed > 0;
    ctx.pid.reset();
    const uint32_t startMs = millis();

    bool found = false;
    while (millis() - startMs < crossLineTimeoutMs)
    {
        driveStep(ctx, speed, startMs, steer);
        const bool onCrossLine = (lineStatus == STOP);
        if (onCrossLine == (edge == CrossLine::Reach))
        {
            found = true;
            break;
        }
        if (!missionWait(5))
        {
            return false;
        }
    }
    if (!found)
    {
        brakeDrive(ctx);
        Serial.printf("Mission: cross line not %s within %lu ms\n",
                      edge == CrossLine::Reach ? "reached" : "left", (unsigned long)crossLineTimeoutMs);
        return false;
    }

    const uint32_t extraStartMs = millis();
    while (millis() - extraStartMs < extraMs)
    {
        driveStep(ctx, speed, startMs, steer);
        if (!missionWait(5))
        {
            return false;
        }
    }
    brakeDrive(ctx);
    return true;
}

template <size_t N>
bool shootSeries(const int (&rpms)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        shootSequence(false, rpms[i]); // a missed shot does not stop the run
        if (missionAborted())
        {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Checkpoint actions. Return false only when the robot should stop (or Stop was pressed).
// ---------------------------------------------------------------------------

bool distributionCenterAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    updateBatteryCompensation();

    // Back off the cross line, lower the arm, drive forward over the cross line, back again, raise the arm.
    if (!driveFromCrossLine(ctx, -missionDriveSpeed, CrossLine::Leave, distributionBackOffExtraMs))
        return false;

    ShieldServo &arm = shield.servo(distributionServo);
    arm.setAngleRange(distributionServoRangeDeg);
    arm.setAngle(distributionServoDownDeg);
    if (!missionWait(distributionServoSettleMs))
        return false;

    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Reach, distributionForwardExtraMs))
        return false;
    if (!missionWait(distributionServoSettleMs))
        return false;

    if (!driveFromCrossLine(ctx, -missionDriveSpeed, CrossLine::Reach, distributionReturnExtraMs))
        return false;

    arm.setAngle(distributionServoUpDeg);
    return missionWait(distributionFinalWaitMs);
}

bool loadRevolverAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    for (int i = 0; i < revolverLoadTurns; ++i)
    {
        turnUntilPackage(); // a timeout does not stop the run
        if (!missionWait(revolverLoadPauseMs))
            return false;
    }
    return true;
}

bool firstShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    return shootSeries(firstShootRpms);
}

bool turnAndShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    if (!missionWait(200))
        return false;
    if (!perform180Turn(ctx.qtr, ctx.motorL, ctx.motorR))
        return false; // turn timed out (or Stop): the robot stops
    ctx.pid.reset();
    if (!missionWait(200))
        return false;
    return shootSeries(turnShootRpms);
}

bool servoCheckpointAction(MissionContext &ctx)
{
    brakeDrive(ctx); // previously the robot kept driving at its last speed while shooting
    const int rpms[] = {servoCheckpointShootRpm};
    return shootSeries(rpms);
}

bool finishRunAction(MissionContext &ctx)
{
    allignOnCheckpoint(ctx.pid, ctx.qtr, ctx.motorL, ctx.motorR);
    brakeDrive(ctx);
    robotMotionEnabled = false; // run complete
    return true;
}

// ---------------------------------------------------------------------------
// The mission: which action runs at which checkpoint. Checkpoint numbers are in config.h.
// ---------------------------------------------------------------------------

struct CheckpointAction
{
    int checkpoint;
    bool (*run)(MissionContext &);
    const char *name;
};

const CheckpointAction mission[] = {
    {distributionCenter, distributionCenterAction, "distribution center"},
    {revolverLoadCheckpoint, loadRevolverAction, "load revolver"},
    {firstShootCheckpoint, firstShootAction, "shoot x3"},
    {turnCheckpoint, turnAndShootAction, "180 turn + shoot x3"},
    {servoCheckpoint, servoCheckpointAction, "shoot x1"},
    {finalCheckpoint, finishRunAction, "finish"},
};

} // namespace

bool runCheckpointAction(int checkpointNumber, MissionContext &ctx)
{
    const CheckpointAction *action = nullptr;
    for (const CheckpointAction &entry : mission)
    {
        if (entry.checkpoint == checkpointNumber)
        {
            action = &entry;
            break;
        }
    }

    Serial.printf("Checkpoint %d%s%s\n", checkpointNumber, action ? ": " : "", action ? action->name : "");

    if (stopAtCheckpoints)
    {
        brakeDrive(ctx);
        if (!missionWait(checkpointStopPauseMs))
            return false;
    }

    bool ok = true;
    if (action != nullptr)
    {
        ok = action->run(ctx);
    }

    if (action != nullptr || stopAtCheckpoints)
    {
        ctx.pid.reset(); // resume line following with a clean controller
    }
    return ok;
}
