#pragma once
#include "pinout.h"
#include "line-follower.h"
#include <Arduino.h>

extern float pidControlKp;
extern float pidControlKi;
extern float pidControlKd;
extern int pidError;
extern float controlSignal; // Variable zur Speicherung des Steuersignals für die Motoren.

extern int defaultMotorSpeed;
extern int rightMotorSpeed;
extern int leftMotorSpeed;
extern int minPWMtoMoveMotor; // Deadzone of the DC Motor to overcome static friction
extern bool enableMotorDeadzoneCompensation;

extern int driveLoopHzTarget;
extern int telemetryHzTarget;

extern int checkpointCounter;
extern int lastCheckpointTimeMs;
extern bool stopAtCheckpoints;
extern bool robotMotionEnabled;
extern bool brakeWhenStopped;

// Supply rails and M1/M2 current monitoring (sampled in the web task).
extern bool enableSupplyAndM1M2CurrentMonitoring;
extern float batteryVoltageV;
extern float servoRailVoltageV;
extern float motorCurrentsA[2]; // index 0..1 -> motor 1..2

extern float driveLoopHzMeasured;
extern float driveLoopExecUsMeasured;
extern float telemetryHzMeasured;
extern float telemetryExecUsPeak;

extern uint16_t qtrSensorValues[QTR_SENSOR_COUNT]; // Array zur Speicherung der aktuellen Sensordaten
extern bool qtrSensorStatus[QTR_SENSOR_COUNT];     // Array zur Speicherung, ob die Sensoren die Linie erkennen

extern enum LineStatus lineStatus;    // Aktueller Linienstatus des Roboters
extern enum LineStatus oldLineStatus; // Vorheriger Linienstatus zum Erkennen von Zustandsänderungen
