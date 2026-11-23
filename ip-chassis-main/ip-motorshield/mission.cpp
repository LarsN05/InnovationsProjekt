#include "mission.h"

#include "config.h"
#include "movement-functions.h"
#include "shared-globals.h"
#include "src/motorshield/motorshield.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "buzzer.h"

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

// Fires every shot of the shooting plan (config.h) for this cross line. afterTurn selects the shots
// before or after the 180 degree turn at turnCheckpoint. A missed shot does not stop the run.
bool shootPlannedShots(int checkpointNumber, bool afterTurn)
{
    for (const PlannedShot &planned : shotPlan)
    {
        if (planned.checkpoint != checkpointNumber || planned.afterTurn != afterTurn)
            continue;
        if (planned.balconyHeight < 0 || planned.balconyHeight >= balconyHeightCount)
        {
            Serial.printf("Shot plan: balcony height %d does not exist, shot skipped\n", planned.balconyHeight);
            continue;
        }
        const ShotSetting &shot = shotSettings[planned.house][planned.balconyHeight];
        Serial.printf("Shot: house %d, balcony %d, %s slide, %d rpm\n", planned.house + 1, planned.balconyHeight,
                      shot.leftSlide ? "left" : "right", shot.rpm);
        shootSequence(shot.leftSlide, shot.rpm);
        if (missionAborted())
            return false;
    }
    return true;
}

bool hasPlannedShots(int checkpointNumber)
{
    for (const PlannedShot &planned : shotPlan)
    {
        if (planned.checkpoint == checkpointNumber)
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Checkpoint actions. Return false only when the robot should stop (or Stop was pressed).
// ---------------------------------------------------------------------------

bool distributionCenterAction(MissionContext &ctx)
{
    // 1. Stop on the cross line. Rule: stand still at the marker for at least 3 s before the flap is triggered.
    brakeDrive(ctx);
    if (!missionWait(distributionStandStillMs))
        return false;
    updateBatteryCompensation();

    // 2. Back off the cross line
    if (!driveFromCrossLine(ctx, -missionDriveSpeed, CrossLine::Leave, distributionBackOffExtraMs))
        return false;

    // 3. Extend the arm
    ShieldServo &arm = shield.servo(distributionServo);
    arm.setAngleRange(distributionServoRangeDeg);
    arm.setAngle(distributionServoDownDeg);
    if (!missionWait(distributionServoSettleMs))
        return false;

    // 4. Drive forward over the cross line with the arm extended, so the cubes fall.
    //    This ends past the cross line, so the line following does not count it a second time.
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Reach, distributionForwardExtraMs))
        return false;
    if (!missionWait(distributionDropWaitMs))
        return false;

    // 5. Retract the arm, then line following continues to the next checkpoint
    arm.setAngle(distributionServoUpDeg);
    return missionWait(distributionServoSettleMs);
}

// Balcony cross line: stop, shoot the planned shots, continue.
bool balconyShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    return shootPlannedShots(checkpointCounter, false);
}

// Top cross line: shoot Haus 2, turn 180 degrees, shoot Haus 3.
bool turnAndShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    if (!shootPlannedShots(turnCheckpoint, false))
        return false;
    if (!missionWait(200))
        return false;
    if (enableMissionSounds)
        playSound(Sound::Turn);
    if (!perform180Turn(ctx.qtr, ctx.motorL, ctx.motorR))
        return false; // turn timed out (or Stop): the robot stops
    ctx.pid.reset();

    // After the turn the sensors may still be on the cross line: drive off it before shooting,
    // so the line following does not count it a second time (does nothing if the robot is already off it).
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Leave, 0))
        return false;
    brakeDrive(ctx);
    if (!missionWait(200))
        return false;
    return shootPlannedShots(turnCheckpoint, true);
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
// Every other cross line that appears in shotPlan (config.h) runs balconyShootAction.
// ---------------------------------------------------------------------------

struct CheckpointAction
{
    int checkpoint;
    bool (*run)(MissionContext &);
    const char *name;
    Sound sound; // played when the action starts (if enableMissionSounds)
};

const CheckpointAction mission[] = {
    {distributionCenter, distributionCenterAction, "distribution center", Sound::Distribution},
    {turnCheckpoint, turnAndShootAction, "shoot, 180 turn, shoot", Sound::Checkpoint},
    {finalCheckpoint, finishRunAction, "finish", Sound::Finish},
};

const CheckpointAction balconyAction = {0, balconyShootAction, "balcony shots", Sound::Checkpoint};

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
    if (action == nullptr && hasPlannedShots(checkpointNumber))
    {
        action = &balconyAction;
    }

    Serial.printf("Checkpoint %d%s%s\n", checkpointNumber, action ? ": " : "", action ? action->name : "");
    if (enableMissionSounds)
    {
        playSound(action ? action->sound : Sound::Checkpoint);
    }

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
