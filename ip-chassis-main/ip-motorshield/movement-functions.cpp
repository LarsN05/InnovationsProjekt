#include "movement-functions.h"
#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"

constexpr uint32_t minTurnTimeMs = 400; //ensures a minimal turn of atleast 120°
constexpr uint32_t turnTimeOutMs = 1000; //constant for the maximum time a turn should be attempted for

bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR){
    turnLeft(motorL, motorR);


    uint32_t startOfTurn = millis(); //variable for detecting if the turn failed (took too long)
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.readLineBlack(sensorValues);

    //first turn until the sensor is not on the line anymore
    while((sensorValues[2] > BLACKLINE_THRESHOLD) && millis() - startOfTurn < turnTimeOutMs){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    //turn until the it detects the line again with minturntime to make sure it doesn't stop at a perpendicular line
    while(((sensorValues[2] < BLACKLINE_THRESHOLD) || (millis() - startOfTurn < minTurnTimeMs)) && millis() - startOfTurn < turnTimeOutMs){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    motorL.brake();
    motorR.brake();

    bool turnSuccess = millis() - startOfTurn < 1000;

    if(!turnSuccess){
      Serial.println("180-turn error");
    }

    return turnSuccess;
}