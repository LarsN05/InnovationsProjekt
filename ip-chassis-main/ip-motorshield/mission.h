#pragma once

#include <QTRSensors.h>
#include "line-follower.h"
#include "src/motorshield/motor.h"

// Everything a checkpoint action needs to drive the robot.
struct MissionContext
{
    PIDController &pid;
    QTRSensors &qtr;
    Motor &motorL;
    Motor &motorR;
};

// Runs the action configured for this checkpoint (see the table in mission.cpp).
// Returns false if a step failed and the robot should stop; also false after Stop was pressed.
// Checkpoints without an action return true immediately.
bool runCheckpointAction(int checkpointNumber, MissionContext &ctx);
