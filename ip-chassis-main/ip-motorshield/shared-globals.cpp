#include "shared-globals.h"

float pidControlKp = 0.32;
float pidControlKi = 0.4;
float pidControlKd = 0.0;
int pidError = 0;
float controlSignal; // Variable zur Speicherung des Steuersignals für die Motoren.

int defaultMotorSpeed = 960;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
bool enableMotorDeadzoneCompensation = false;
int minPWMtoMoveMotor = 560; // Deadzone of the DC Motor to overcome static friction

int driveLoopHzTarget = 40;
int telemetryHzTarget = 20;

int checkpointCounter = 0;
int lastCheckpointTimeMs = 0;

bool stopAtCheckpoints = false;
bool robotMotionEnabled = false;
bool brakeWhenStopped = false;

bool enableSupplyAndM1M2CurrentMonitoring = false;
float batteryVoltageV = NAN;
float servoRailVoltageV = NAN;
float motorCurrentsA[2] = {NAN, NAN};

float driveLoopHzMeasured = 0.0f;
float driveLoopExecUsMeasured = 0.0f;
float telemetryHzMeasured = 0.0f;
float telemetryExecUsPeak = 0.0f;

uint16_t qtrSensorValues[QTR_SENSOR_COUNT]; // Array zur Speicherung der aktuellen Sensordaten
bool qtrSensorStatus[QTR_SENSOR_COUNT];     // Array zur Speicherung, ob die Sensoren die Linie erkennen

enum LineStatus lineStatus = NORMAL;        // Aktueller Linienstatus des Roboters
enum LineStatus oldLineStatus = lineStatus; // Vorheriger Linienstatus zum Erkennen von Zustandsänderungen
