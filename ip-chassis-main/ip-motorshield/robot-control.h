#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

#include "line-follower.h"
#include "src/motorshield/motorshield.h"

// High-level robot state. The drive loop (core 1) owns the motors; the web
// task (core 0) only posts requests, which are applied in updateRobotControl().
enum class RobotMode : uint8_t
{
    Idle,        // motors stopped, waiting for a command
    FollowLine,  // PID line following, runs the mission at every checkpoint
    Manual,      // motor speeds set directly from the web interface
    Paused,      // waiting at a checkpoint (pause / lever action), resumes automatically
    Busy         // blocking manoeuvre: calibration, 180 turn, alignment
};

// What the robot does when it reaches a checkpoint (black stop line).
enum class CheckpointAction : uint8_t
{
    Continue = 0,   // drive on
    Stop = 1,       // stop and wait for Start / Next Stop
    Pause = 2,      // stop for param ms, then drive on
    Turn180 = 3,    // turn around on the line, then drive on
    LeverCycle = 4, // stop, lever to active, hold param ms, lever back to rest, drive on
    LeverSet = 5,   // move lever to param degrees without stopping
    Finish = 6,     // align on the checkpoint and stop
    Count
};

struct CheckpointStep
{
    CheckpointAction action;
    int16_t param;
};

constexpr uint8_t maxMissionSteps = 24;

enum class RobotRequestType : uint8_t
{
    Start,
    Stop,
    NextStop,
    Manual,            // a = left speed, b = right speed
    Calibrate,
    Turn180,
    Align,
    SetCheckpoint,     // a = new checkpoint counter
    Lever,             // a = angle in degrees, -1 releases the servo
};

enum class RobotEventType : uint8_t
{
    ArrivedAtStop,     // arg = checkpoint number
    Checkpoint,        // arg = checkpoint number
    Finished,          // arg = checkpoint number
    TurnFailed,
    AlignFailed,
    CalibrationDone,
    ManualTimeout,
};

struct RobotHardwareSettings
{
    bool invertLeftMotor;
    bool invertRightMotor;
    bool swapMotors;
    bool invertSteering;
};

extern volatile RobotMode robotMode;
extern volatile bool stopAtNextCheckpoint;
extern volatile int leverArmAngleDeg; // -1 = released (no pulses)
extern volatile int linePosition;     // 0..4000, 2000 = centered
extern RobotHardwareSettings hardwareSettings;

void initRobotControl(Motorshield &shield);

// Called from loop() on core 1 before the drive loop.
void updateRobotControl(PIDController &pid, QTRSensors &qtr);

// Called by the drive loop each time a new checkpoint is detected.
void handleCheckpoint(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR);

Motor &leftMotor();
Motor &rightMotor();

// Thread-safe, callable from the web task.
bool postRobotRequest(RobotRequestType type, int a = 0, int b = 0);
bool popRobotEvent(RobotEventType &type, int &arg);

uint8_t getMission(CheckpointStep *steps);
void setMission(const CheckpointStep *steps, uint8_t count);
void resetMissionToDefault();
void applyHardwareSettings();

const char *robotModeName(RobotMode mode);

// Persist PID/speed parameters, hardware settings and mission in flash.
void loadSettings(PIDController &pid);
bool saveSettings();
void clearSavedSettings();
