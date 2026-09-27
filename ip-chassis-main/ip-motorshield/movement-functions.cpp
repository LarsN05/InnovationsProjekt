#include "movement-functions.h"
#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"



bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR){
    turnLeft(motorL, motorR);

    uint32_t startOfTurn = millis(); //variable for detecting if the turn failed (took too long)
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.readLineBlack(sensorValues);

    //first turn until the sensor is not on the line anymore
    while((sensorValues[2] > BLACKLINE_THRESHOLD) && millis() - startOfTurn < 1000){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    //turn until the it detects the line again
    while((sensorValues[2] < BLACKLINE_THRESHOLD) && millis() - startOfTurn < 1000){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    motorL.brake()
    motorR.brake()

    bool turnSuccess = millis() - startOfTurn < 1000;

    if(!turnSuccess){
      Serial.println("180-turn error");
    }

    return turnSuccess
}