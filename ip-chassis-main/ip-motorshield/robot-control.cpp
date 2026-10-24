#include "robot-control.h"

#include "config.h"
#include "movement-functions.h"
#include "onboard-led.h"
#include "qtr-sensor-util.h"
#include "shared-globals.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

volatile RobotMode robotMode = RobotMode::Idle;
volatile bool stopAtNextCheckpoint = false;
volatile int leverArmAngleDeg = leverArmRestDeg;
volatile int linePosition = 2000;
RobotHardwareSettings hardwareSettings = {
    defaultInvertLeftMotor,
    defaultInvertRightMotor,
    defaultSwapMotors,
    defaultInvertSteering,
};

namespace
{
struct RobotRequest
{
    RobotRequestType type;
    int a;
    int b;
};

struct RobotEvent
{
    RobotEventType type;
    int arg;
};

// The web interface sends manual commands every 100 ms while a drive button is held.
constexpr uint32_t manualTimeoutMs = 400;
// Time the lever needs to swing back to rest before driving on after a lever cycle.
constexpr uint32_t leverReturnSettleMs = 500;

Motorshield *activeShield = nullptr;
QueueHandle_t requestQueue = nullptr;
QueueHandle_t eventQueue = nullptr;

portMUX_TYPE missionMux = portMUX_INITIALIZER_UNLOCKED;
CheckpointStep mission[maxMissionSteps];
uint8_t missionLength = 0;

uint32_t lastManualCommandMs = 0;
uint32_t pausedUntilMs = 0;
bool leverReturnPending = false;

void pushEvent(RobotEventType type, int arg = 0)
{
    if (eventQueue == nullptr)
    {
        return;
    }
    RobotEvent event{type, arg};
    xQueueSend(eventQueue, &event, 0);
}

void setMode(RobotMode mode)
{
    robotMode = mode;
    robotMotionEnabled = (mode == RobotMode::FollowLine);
}

void brakeDriveMotors()
{
    leftMotor().brake();
    rightMotor().brake();
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
}

void moveLever(int angleDeg)
{
    leverArmAngleDeg = angleDeg;
    if (angleDeg < 0)
    {
        activeShield->servo(leverArmServo).disable();
    }
    else
    {
        activeShield->servo(leverArmServo).setAngle(angleDeg);
    }
}

void resetPid(PIDController &pid)
{
    pid.integral = 0;
    pid.prevError = 0;
}

// Mode to enter once the current checkpoint action is complete.
RobotMode modeAfterCheckpointAction()
{
    if (stopAtNextCheckpoint)
    {
        stopAtNextCheckpoint = false;
        pushEvent(RobotEventType::ArrivedAtStop, checkpointCounter);
        return RobotMode::Idle;
    }
    return RobotMode::FollowLine;
}

void pauseThenContinue(uint32_t durationMs)
{
    brakeDriveMotors();
    pausedUntilMs = millis() + durationMs;
    setMode(RobotMode::Paused);
}

CheckpointStep missionStepFor(int checkpoint)
{
    CheckpointStep step{CheckpointAction::Continue, 0};
    portENTER_CRITICAL(&missionMux);
    if (checkpoint >= 1 && checkpoint <= missionLength)
    {
        step = mission[checkpoint - 1];
    }
    portEXIT_CRITICAL(&missionMux);
    return step;
}

void handleRequest(const RobotRequest &request, PIDController &pid, QTRSensors &qtr)
{
    // Any new drive command cancels a lever cycle in progress; put the lever back first.
    if (leverReturnPending && request.type != RobotRequestType::Lever && request.type != RobotRequestType::SetCheckpoint)
    {
        leverReturnPending = false;
        moveLever(leverArmRestDeg);
    }

    switch (request.type)
    {
    case RobotRequestType::Start:
        stopAtNextCheckpoint = false;
        resetPid(pid);
        setMode(RobotMode::FollowLine);
        break;

    case RobotRequestType::Stop:
        stopAtNextCheckpoint = false;
        setMode(RobotMode::Idle);
        brakeDriveMotors();
        break;

    case RobotRequestType::NextStop:
        stopAtNextCheckpoint = true;
        resetPid(pid);
        setMode(RobotMode::FollowLine);
        break;

    case RobotRequestType::Manual:
        if (robotMode == RobotMode::Busy)
        {
            break;
        }
        stopAtNextCheckpoint = false;
        setMode(RobotMode::Manual);
        lastManualCommandMs = millis();
        leftMotorSpeed = constrain(request.a, -PWM_MAX_VALUE, PWM_MAX_VALUE);
        rightMotorSpeed = constrain(request.b, -PWM_MAX_VALUE, PWM_MAX_VALUE);
        leftMotor().setSpeed(leftMotorSpeed);
        rightMotor().setSpeed(rightMotorSpeed);
        break;

    case RobotRequestType::Calibrate:
        setMode(RobotMode::Busy);
        calibrateSensors(qtr, leftMotor(), rightMotor());
        resetPid(pid);
        setMode(RobotMode::Idle);
        pushEvent(RobotEventType::CalibrationDone);
        break;

    case RobotRequestType::Turn180:
        setMode(RobotMode::Busy);
        brakeDriveMotors();
        if (!perform180Turn(qtr, leftMotor(), rightMotor()))
        {
            pushEvent(RobotEventType::TurnFailed);
        }
        resetPid(pid);
        setMode(RobotMode::Idle);
        break;

    case RobotRequestType::Align:
        setMode(RobotMode::Busy);
        if (!allignOnCheckpoint(pid, qtr, leftMotor(), rightMotor()))
        {
            pushEvent(RobotEventType::AlignFailed);
        }
        brakeDriveMotors();
        resetPid(pid);
        setMode(RobotMode::Idle);
        break;

    case RobotRequestType::SetCheckpoint:
        checkpointCounter = max(request.a, 0);
        break;

    case RobotRequestType::Lever:
        moveLever(request.a < 0 ? -1 : constrain(request.a, 0, static_cast<int>(leverArmAngleRangeDeg)));
        break;
    }
}

void putMissionStep(CheckpointStep *steps, uint8_t &count, int checkpoint, CheckpointAction action, int16_t param = 0)
{
    if (checkpoint < 1 || checkpoint > maxMissionSteps)
    {
        return;
    }
    while (count < checkpoint)
    {
        steps[count++] = {CheckpointAction::Continue, 0};
    }
    steps[checkpoint - 1] = {action, param};
}
}

Motor &leftMotor()
{
    return activeShield->motor(hardwareSettings.swapMotors ? 2 : 1);
}

Motor &rightMotor()
{
    return activeShield->motor(hardwareSettings.swapMotors ? 1 : 2);
}

void applyHardwareSettings()
{
    activeShield->motor(1).setInverted(hardwareSettings.swapMotors ? hardwareSettings.invertRightMotor : hardwareSettings.invertLeftMotor);
    activeShield->motor(2).setInverted(hardwareSettings.swapMotors ? hardwareSettings.invertLeftMotor : hardwareSettings.invertRightMotor);
}

void resetMissionToDefault()
{
    // The competition run: turn around at the turn checkpoint, park at the final one.
    CheckpointStep steps[maxMissionSteps];
    uint8_t count = 0;
    putMissionStep(steps, count, turnCheckpoint, CheckpointAction::Turn180);
    putMissionStep(steps, count, finalCheckpoint, CheckpointAction::Finish);
    setMission(steps, count);
}

uint8_t getMission(CheckpointStep *steps)
{
    portENTER_CRITICAL(&missionMux);
    const uint8_t count = missionLength;
    memcpy(steps, mission, sizeof(CheckpointStep) * count);
    portEXIT_CRITICAL(&missionMux);
    return count;
}

void setMission(const CheckpointStep *steps, uint8_t count)
{
    count = min(count, maxMissionSteps);
    portENTER_CRITICAL(&missionMux);
    memcpy(mission, steps, sizeof(CheckpointStep) * count);
    missionLength = count;
    portEXIT_CRITICAL(&missionMux);
}

void initRobotControl(Motorshield &shield)
{
    activeShield = &shield;
    requestQueue = xQueueCreate(16, sizeof(RobotRequest));
    eventQueue = xQueueCreate(16, sizeof(RobotEvent));

    resetMissionToDefault();
    applyHardwareSettings();

    shield.servo(leverArmServo).setAngleRange(leverArmAngleRangeDeg);
    moveLever(leverArmRestDeg);
}

bool postRobotRequest(RobotRequestType type, int a, int b)
{
    if (requestQueue == nullptr)
    {
        return false;
    }
    RobotRequest request{type, a, b};
    return xQueueSend(requestQueue, &request, 0) == pdTRUE;
}

bool popRobotEvent(RobotEventType &type, int &arg)
{
    RobotEvent event;
    if (eventQueue == nullptr || xQueueReceive(eventQueue, &event, 0) != pdTRUE)
    {
        return false;
    }
    type = event.type;
    arg = event.arg;
    return true;
}

void updateRobotControl(PIDController &pid, QTRSensors &qtr)
{
    RobotRequest request;
    while (requestQueue != nullptr && xQueueReceive(requestQueue, &request, 0) == pdTRUE)
    {
        handleRequest(request, pid, qtr);
    }

    const uint32_t nowMs = millis();

    if (robotMode == RobotMode::Manual && nowMs - lastManualCommandMs > manualTimeoutMs)
    {
        // Lost connection or button released without a stop message.
        setMode(RobotMode::Idle);
        brakeDriveMotors();
        pushEvent(RobotEventType::ManualTimeout);
    }

    if (robotMode == RobotMode::Paused && static_cast<int32_t>(nowMs - pausedUntilMs) >= 0)
    {
        if (leverReturnPending)
        {
            leverReturnPending = false;
            moveLever(leverArmRestDeg);
            pausedUntilMs = nowMs + leverReturnSettleMs;
            return;
        }
        resetPid(pid);
        setMode(modeAfterCheckpointAction());
    }
}

void handleCheckpoint(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR)
{
    pushEvent(RobotEventType::Checkpoint, checkpointCounter);
    const CheckpointStep step = missionStepFor(checkpointCounter);

    switch (step.action)
    {
    case CheckpointAction::Stop:
        brakeDriveMotors();
        stopAtNextCheckpoint = false;
        setMode(RobotMode::Idle);
        pushEvent(RobotEventType::ArrivedAtStop, checkpointCounter);
        return;

    case CheckpointAction::Pause:
        pauseThenContinue(max<int16_t>(step.param, 0));
        return;

    case CheckpointAction::LeverCycle:
        pauseThenContinue(max<int16_t>(step.param, 0));
        moveLever(leverArmActiveDeg);
        leverReturnPending = true;
        return;

    case CheckpointAction::LeverSet:
        moveLever(constrain(step.param, 0, static_cast<int>(leverArmAngleRangeDeg)));
        break;

    case CheckpointAction::Turn180:
        setMode(RobotMode::Busy);
        motorL.brake();
        motorR.brake();
        delay(200);
        if (!perform180Turn(qtr, motorL, motorR))
        {
            brakeDriveMotors();
            stopAtNextCheckpoint = false;
            setMode(RobotMode::Idle);
            pushEvent(RobotEventType::TurnFailed);
            blinkWarningPattern(255, 0, 0, 10);
            return;
        }
        // Clean restart of the line following after the turn.
        resetPid(pid);
        delay(200);
        break;

    case CheckpointAction::Finish:
        setMode(RobotMode::Busy);
        allignOnCheckpoint(pid, qtr, motorL, motorR);
        brakeDriveMotors();
        stopAtNextCheckpoint = false;
        setMode(RobotMode::Idle);
        pushEvent(RobotEventType::Finished, checkpointCounter);
        return;

    case CheckpointAction::Continue:
    default:
        if (stopAtCheckpoints && !stopAtNextCheckpoint)
        {
            pauseThenContinue(1000);
            return;
        }
        break;
    }

    if (modeAfterCheckpointAction() == RobotMode::Idle)
    {
        brakeDriveMotors();
        setMode(RobotMode::Idle);
    }
    else
    {
        setMode(RobotMode::FollowLine);
    }
}

const char *robotModeName(RobotMode mode)
{
    switch (mode)
    {
    case RobotMode::Idle:
        return "idle";
    case RobotMode::FollowLine:
        return "follow_line";
    case RobotMode::Manual:
        return "manual";
    case RobotMode::Paused:
        return "paused";
    case RobotMode::Busy:
        return "busy";
    }
    return "unknown";
}

void loadSettings(PIDController &pid)
{
    Preferences prefs;
    if (!prefs.begin("ipbot", true))
    {
        return; // nothing saved yet
    }

    pidControlKp = prefs.getFloat("kp", pidControlKp);
    pidControlKi = prefs.getFloat("ki", pidControlKi);
    pidControlKd = prefs.getFloat("kd", pidControlKd);
    defaultMotorSpeed = prefs.getInt("speed", defaultMotorSpeed);
    pid.kp = pidControlKp;
    pid.ki = pidControlKi;
    pid.kd = pidControlKd;

    hardwareSettings.invertLeftMotor = prefs.getBool("invL", hardwareSettings.invertLeftMotor);
    hardwareSettings.invertRightMotor = prefs.getBool("invR", hardwareSettings.invertRightMotor);
    hardwareSettings.swapMotors = prefs.getBool("swap", hardwareSettings.swapMotors);
    hardwareSettings.invertSteering = prefs.getBool("invS", hardwareSettings.invertSteering);
    applyHardwareSettings();

    if (prefs.isKey("mlen"))
    {
        CheckpointStep steps[maxMissionSteps];
        const uint8_t count = min<uint8_t>(prefs.getUChar("mlen", 0), maxMissionSteps);
        if (count == 0 || prefs.getBytes("mission", steps, sizeof(CheckpointStep) * count) == sizeof(CheckpointStep) * count)
        {
            setMission(steps, count);
        }
    }
    prefs.end();
    Serial.println("Loaded saved settings from flash.");
}

bool saveSettings()
{
    Preferences prefs;
    if (!prefs.begin("ipbot", false))
    {
        return false;
    }

    prefs.putFloat("kp", pidControlKp);
    prefs.putFloat("ki", pidControlKi);
    prefs.putFloat("kd", pidControlKd);
    prefs.putInt("speed", defaultMotorSpeed);
    prefs.putBool("invL", hardwareSettings.invertLeftMotor);
    prefs.putBool("invR", hardwareSettings.invertRightMotor);
    prefs.putBool("swap", hardwareSettings.swapMotors);
    prefs.putBool("invS", hardwareSettings.invertSteering);

    CheckpointStep steps[maxMissionSteps];
    const uint8_t count = getMission(steps);
    prefs.putUChar("mlen", count);
    if (count > 0)
    {
        prefs.putBytes("mission", steps, sizeof(CheckpointStep) * count);
    }
    prefs.end();
    return true;
}

void clearSavedSettings()
{
    Preferences prefs;
    if (prefs.begin("ipbot", false))
    {
        prefs.clear();
        prefs.end();
    }
}
