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
bool driveFromCrossLine(MissionContext &ctx, int speed, CrossLine edge, uint32_t extraMs,
                        uint32_t timeoutMs = crossLineTimeoutMs)
{
    const bool steer = speed > 0;
    ctx.pid.reset();
    const uint32_t startMs = millis();

    bool found = false;
    while (millis() - startMs < timeoutMs)
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
                      edge == CrossLine::Reach ? "reached" : "left", (unsigned long)timeoutMs);
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

// Spins on the spot for durationMs (speed > 0 = turn right, < 0 = turn left), then brakes.
bool turnOnSpotFor(MissionContext &ctx, int speed, uint32_t durationMs)
{
    const int s = compensateForBattery(speed);
    ctx.motorL.setSpeed(s);
    ctx.motorR.setSpeed(-s);
    const bool ok = missionWait(durationMs);
    brakeDrive(ctx);
    return ok;
}

// Follows the line (PID steering) for durationMs, then brakes.
bool followLineFor(MissionContext &ctx, int speed, uint32_t durationMs)
{
    ctx.pid.reset();
    const uint32_t startMs = millis();
    while (millis() - startMs < durationMs)
    {
        driveStep(ctx, speed, startMs, true);
        if (!missionWait(5))
            return false;
    }
    brakeDrive(ctx);
    return true;
}

// Fires every balcony row planned for this cross line (shotPlan in config.h), in order, and every shot
// of each row (shots in config.h). Before a row the robot moves as given by its RowMove.
// The shooter keeps spinning between shots and rows; it only stops after the last row of the plan.
// A missed shot does not stop the run.
bool shootPlannedShots(MissionContext &ctx, int checkpointNumber, bool afterTurn)
{
    constexpr size_t planLength = sizeof(shotPlan) / sizeof(shotPlan[0]);
    for (size_t p = 0; p < planLength; ++p)
    {
        const RowStop &stop = shotPlan[p];
        if (stop.checkpoint != checkpointNumber || stop.afterTurn != afterTurn)
            continue;
        if (stop.row < 0 || stop.row >= balconyRowCount)
        {
            Serial.printf("Shot plan: row %d does not exist, skipped\n", stop.row + 1);
            continue;
        }
        if (stop.move == RowMove::FollowLine)
        {
            updateBatteryCompensation();
            if (!followLineFor(ctx, shotDriveSpeed, thirdBalconyDriveMs))
                return false;
            if (!missionWait(200)) // let the robot settle before shooting
                return false;
        }
        else if (stop.move == RowMove::TurnRight)
        {
            if (!turnOnSpotFor(ctx, rowTurnSpeed, rowTurnMs))
                return false;
            if (!missionWait(200))
                return false;
        }
        const bool lastRowOfRun = (p == planLength - 1);
        for (int i = 0; i < shotsPerRow[stop.row]; ++i)
        {
            const Shot &shot = shots[stop.house][stop.row][i];
            const bool lastShotOfRun = lastRowOfRun && (i == shotsPerRow[stop.row] - 1);
            Serial.printf("Shot: Haus %d, row %d, shot %d: %d rpm, %s slide\n", stop.house + 1, stop.row + 1, i + 1,
                          shot.rpm, shot.switchRight ? "right" : "left");
            shootSequence(shot.switchRight, shot.rpm, lastShotOfRun);
            if (missionAborted())
                return false;
        }
        if (stop.move == RowMove::TurnRight)
        {
            // turn back to the line direction
            if (!turnOnSpotFor(ctx, -rowTurnSpeed, rowTurnMs))
                return false;
            if (!missionWait(200))
                return false;
        }
    }
    return true;
}

bool hasPlannedShots(int checkpointNumber, bool afterTurn = false)
{
    for (const RowStop &stop : shotPlan)
    {
        if (stop.checkpoint == checkpointNumber && stop.afterTurn == afterTurn)
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

    // 4. Drive forward over the cross line with the arm extended: this triggers the distribution center
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Reach, distributionForwardExtraMs))
        return false;
    if (!missionWait(distributionDropWaitMs))
        return false;

    // 5. Drive back until the cross line is reached again, then a bit further
    if (!driveFromCrossLine(ctx, -missionDriveSpeed, CrossLine::Reach, distributionReturnExtraMs))
        return false;

    // 6. Retract the arm
    arm.setAngle(distributionServoUpDeg);
    if (!missionWait(distributionServoSettleMs))
        return false;

    // 7. Drive forward over the cross line and off it, so the line following does not count it a second time.
    //    Then line following continues to the next checkpoint.
    if (!driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Reach, 0))
        return false;
    return driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Leave, 0);
}

// Balcony cross line: brake on the line and fire the planned shots, then continue.
bool balconyShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    if (!missionWait(200))
        return false;
    return shootPlannedShots(ctx, checkpointCounter, false);
}

// After the 180 turn: drive forward (with line steering) until the sensors see the cross line, then align:
// forward past it for turnAlignWiggleMs, back slowly until the sensors are on the cross line again, brake.
// Returns false only on Stop; if a line is not found the robot brakes and the run continues.
// findOnly = true: only step 1 (drive forward until the sensors are on the cross line), no wiggle.
bool alignOnCrossLineAfterTurn(MissionContext &ctx, bool findOnly = false)
{
    // 1. Find the cross line (does not move if the sensors are already on it)
    ctx.qtr.readLineBlack(qtrSensorValues);
    updateLineStatus();
    ctx.pid.reset();
    uint32_t startMs = millis();
    while (lineStatus != STOP)
    {
        if (millis() - startMs >= turnAlignSearchMs)
        {
            brakeDrive(ctx);
            Serial.println("Mission: no cross line found after the turn, shooting Haus 3 row 1 from here");
            return true;
        }
        driveStep(ctx, turnAlignSpeed, startMs, true);
        if (!missionWait(5))
            return false;
    }
    brakeDrive(ctx);
    if (findOnly)
        return true;
    if (!missionWait(150))
        return false;

    // 2. Forward past the cross line (line steering straightens the robot)
    if (!followLineFor(ctx, turnAlignSpeed, turnAlignWiggleMs))
        return false;
    if (!missionWait(150))
        return false;

    // 3. Back slowly until the sensors are on the cross line again
    startMs = millis();
    while (millis() - startMs < turnAlignBackTimeoutMs)
    {
        driveStep(ctx, -turnAlignSpeed, startMs, false);
        if (lineStatus == STOP)
        {
            brakeDrive(ctx);
            return true;
        }
        if (!missionWait(5))
            return false;
    }
    brakeDrive(ctx);
    Serial.println("Mission: cross line not found again while aligning, shooting Haus 3 row 1 from here");
    return true;
}

// Top cross line (7): Haus 2 row 2 on the line, ~10 degree turn, Haus 2 row 3, turn back,
// 180 degree turn, align on the middle cross line under the sensors, Haus 3 row 1, drive off that line
// (so it is not counted). Haus 3 rows 2 and 3 follow at the next cross line.
bool turnAndShootAction(MissionContext &ctx)
{
    brakeDrive(ctx);
    if (!missionWait(200))
        return false;
    if (!shootPlannedShots(ctx, turnCheckpoint, false))
        return false;
    if (!missionWait(200))
        return false;

    // Make sure the sensors are on the top cross line before turning (does nothing if they already are)
    updateBatteryCompensation();
    if (!driveFromCrossLine(ctx, -missionDriveSpeed, CrossLine::Reach, 0, thirdBalconyDriveMs + 2000))
        return false;
    if (!missionWait(200))
        return false;

    if (enableMissionSounds)
        playSound(Sound::Turn);
    if (!perform180Turn(ctx.qtr, ctx.motorL, ctx.motorR))
        return false; // turn timed out (or Stop): the robot stops
    brakeDrive(ctx);
    ctx.pid.reset();
    if (!missionWait(200))
        return false;

    if (hasPlannedShots(turnCheckpoint, true))
    {
        // Haus 3 row 1 on the middle cross line
        if (!alignOnCrossLineAfterTurn(ctx))
            return false;
        if (!missionWait(200))
            return false;
        if (!shootPlannedShots(ctx, turnCheckpoint, true))
            return false;
    }
    else
    {
        // No shots after the turn (Haus 3 skipped): only make sure the sensors are on the middle cross line,
        // so driving off it below keeps it from being counted. Then straight back to the finish.
        if (!alignOnCrossLineAfterTurn(ctx, true))
            return false;
    }

    // Drive off the middle cross line so the line following does not count it
    return driveFromCrossLine(ctx, missionDriveSpeed, CrossLine::Leave, 0);
}

bool finishRunAction(MissionContext &ctx)
{
    adjustSpeed(0); // shooter off (it normally already stopped after the last shot)
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
