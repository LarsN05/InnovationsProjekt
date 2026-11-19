#include "mission.h"

#include "config.h"
#include "movement-functions.h"
#include "shared-globals.h"
#include "src/motorshield/motorshield.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"

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

// Shoots once per balcony height at the given house. Slide and rpm of each shot come from shotSettings (config.h).
template <size_t N>
bool shootSeries(HouseIndex house, const int (&balconyHeights)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        const int height = balconyHeights[i];
        if (height < 0 || height >= balconyHeightCount)
        {
            Serial.printf("shootSeries: balcony height %d does not exist, shot skipped\n", height);
            continue;
        }
        const ShotSetting &shot = shotSettings[house][height];
        shootSequence(shot.leftSlide, shot.rpm); // a missed shot does not stop the run
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

    // Rule: the robot has to stand still at the marker for at least 3 s before the flap is triggered.
    if (!missionWait(distributionStandStillMs))
        return false;

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
    if (!missionWait(distributionFinalWaitMs))
        return false;

    // The return move ends behind the cross line. Drive over it here, so the line following
    // does not count the distribution center a second time.
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Reach, 0))
        return false;
    return driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Leave, 0);
}

bool firstShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    return shootSeries(firstShootHouse, firstShootBalconyHeights);
}

bool turnAndShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    if (!missionWait(200))
        return false;
    if (!perform180Turn(ctx.qtr, ctx.motorL, ctx.motorR))
        return false; // turn timed out (or Stop): the robot stops
    ctx.pid.reset();

    // After the turn the sensors may still be on the cross line: drive off it before shooting,
    // so the line following does not count it a second time (does nothing if the robot is already off it).
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Leave, 0))
        return false;
    if (!missionWait(200))
        return false;
    return shootSeries(turnShootHouse, turnShootBalconyHeights);
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
    {firstShootCheckpoint, firstShootAction, "shoot x3"},
    {turnCheckpoint, turnAndShootAction, "180 turn + shoot x3"},
    {finalCheckpoint, finishRunAction, "finish"},
};

} // namespace

#if ALIGN_TEST_MODE
// Alignment test: run only allignOnCheckpoint() at every cross line, report, then stop.
static bool runAlignTest(int checkpointNumber, MissionContext &ctx)
{
    Serial.printf("Align test #%d: cross line detected, aligning...\n", checkpointNumber);
    const uint32_t startMs = millis();
    const bool aligned = allignOnCheckpoint(ctx.pid, ctx.qtr, ctx.motorL, ctx.motorR);
    const uint32_t durationMs = millis() - startMs;
    brakeDrive(ctx);

    if (!robotMotionEnabled)
    {
        Serial.println("Align test: aborted by Stop");
        return true;
    }

    Serial.printf("Align test #%d: %s after %lu ms. Sensor values at the end position:\n",
                  checkpointNumber, aligned ? "OK" : "FAILED (timeout)", (unsigned long)durationMs);
    printSensorValues(ctx.qtr);

    robotMotionEnabled = false; // stop and wait for Start in the web interface
    ctx.pid.reset();
    blinkWarningPattern(aligned ? 0 : 255, aligned ? 255 : 0, 0, 3); // green = OK, red = failed
    return true;
}
#endif

bool runCheckpointAction(int checkpointNumber, MissionContext &ctx)
{
#if ALIGN_TEST_MODE
    return runAlignTest(checkpointNumber, ctx);
#endif

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
