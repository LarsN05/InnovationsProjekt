#pragma once
#include "pinout.h"
#include <Arduino.h>

extern uint16_t qtrSensorValues[QTR_SENSOR_COUNT]; // Array zur Speicherung der aktuellen Sensordaten
extern bool qtrSensorStatus[QTR_SENSOR_COUNT];     // Array zur Speicherung, ob die Sensoren die Linie erkennen


