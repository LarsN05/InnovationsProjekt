#include <Arduino.h>

#include <QTRSensors.h>

#include "onboard-led.h"
#include "config.h"
#include "qtr-sensor-util.h"
#include "shared-globals.h"
#include "src/motorshield/motorshield.h"

// Dies ist das IP26-Mechatronik-Testskript für die erhaltenen Mechatronik-Komponenten,
// das IP-Motorshield und den ESP32.
//
// Um zu verifizieren, dass alle Komponenten des Starter-Kits funktionieren,
// folgen Sie den Anweisungen im Kapitel "Mechatronik-Test" des Quickstart Guides auf Moodle.
//
// **Wichtig:** Bevor Sie den Test durchführen, stellen Sie sicher, dass alle Komponenten
// angeschlossen sind und die Polarität korrekt ist.
// Öffnen Sie anschliessend den Serial Monitor.

Motorshield shield;
QTRSensors qtr;

constexpr float batteryWarn40PercentV = 13.1f;
constexpr float batteryWarn20PercentV = 12.9f;
bool test = true;

void setup()
{
  Serial.begin(115200);
  delay(3000);

  Serial.println("Motorshield firmware starting up...");
  initOnboardLed();
  setOnboardLedColor(255, 0, 0);

  if (!shield.begin())
  {
    Serial.println("Motorshield warning: not all I2C chips responded.");
  }

  const float batteryVoltageV = shield.batteryVoltage(8);
  Serial.printf("Battery voltage: %.2f V\n", batteryVoltageV);

  if (batteryVoltageV < batteryWarn20PercentV)
  {
    Serial.println("Battery warning: below 20%, blinking red 5x.");
    blinkWarningPattern(255, 0, 0, 5);
  }
  else if (batteryVoltageV < batteryWarn40PercentV)
  {
    Serial.println("Battery warning: below 40%, blinking orange 5x.");
    blinkWarningPattern(255, 100, 0, 5);
  }

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSOR_PINS, QTR_SENSOR_COUNT);
  qtr.setTimeout(1000); // Reduced from default 2500 to increase control-loop frequency.

  delay(200);
  setOnboardLedColor(0, 255, 0);
  delay(200);
  setOnboardLedColor(0, 0, 255);
  delay(200);

  clearOnboardLed();
}

void loop()
{

  while(test)
  { 
    //================================
    // Test Drive Motors
    //================================
    // The onboard LED will glow purple
    // The yellow drive motors will simultaneously move:
    // - Forward
    // - Backward
    // - Ramp to full speed 
    // The onboard LED will blink purple three times. 

    setOnboardLedColor(255, 0, 255); //Purple

    delay(5000);

    Serial.print("Forrward Drive Motors \n");
    delay(1000);
    shield.motor(1).setSpeed(2000);
    shield.motor(2).setSpeed(2000);
    delay(1500);
    float Current1 = shield.motor(1).readCurrentAmps();
    float Current2 = shield.motor(2).readCurrentAmps();
    Serial.printf("Current through motor1 is %f A \n", Current1);
    Serial.printf("Current through motor2 is %f A \n", Current2);
    delay(1500);


    Serial.print("Backward Drive Motors\n");
    shield.motor(1).setSpeed(-2000);
    shield.motor(2).setSpeed(-2000);
    delay(3000);

    Serial.print("Ramp Drive Motors \n");
    delay(1000);
    for (unsigned int i = 0; i < 100; i++){
      shield.motor(1).setSpeed(100 + i*40);
      shield.motor(2).setSpeed(100 + i*40);
      delay(50);
    }
    delay(1000);
    shield.motor(1).stop();
    shield.motor(2).stop();

    clearOnboardLed();
    blinkWarningPattern(255, 0, 255, 3);

    //=================================
    // Test PCA Motors
    //=================================
    // The onboard LED will glow orange
    // The yellow drive motors will move individually:
    // - Forward
    // - Backward
    // - Ramp to full speed 
    // The onboard LED will blink orange three times. 

    setOnboardLedColor(255, 165, 0); //orange

    // Run forward individualy
    for (unsigned int i = 3; i <= 4; i++){
      Serial.printf("Testing motor %u \n", i);
      shield.motor(i).setSpeed(3000);
      delay(2000);
      shield.motor(i).setSpeed(-3000);
      delay(2000);
      for (unsigned int j = 0; j < 100; j++){
        shield.motor(i).setSpeed(100 + j*40);
        delay(50);
      }
      delay(1000);
      shield.motor(i).stop();
    }

    clearOnboardLed();
    blinkWarningPattern(255, 165, 0, 3);

    //=================================
    // Servo Test
    //=================================
    // The onboard LED will glow green.
    // Each servo individually moves from 0 to 180 twice. 
    // The onboard LED will blink green three times.

    setOnboardLedColor(0, 255, 0); //Green

    //Specify the angle range for each individual servo
    shield.servo(1).setAngleRange(180);
    shield.servo(2).setAngleRange(180);
    shield.servo(3).setAngleRange(270);

    for (unsigned int i = 1; i<=3; i++){
      Serial.printf("Servo %u is moving\n", i);
      shield.servo(i).setAngle(0);
      delay(1000);
      shield.servo(i).setAngle(180);
      delay(1000);
      shield.servo(i).setAngle(0);
      delay(1000);
      shield.servo(i).setAngle(180);
      delay(1000);
      shield.servo(i).setAngle(0);
    }

    clearOnboardLed();
    blinkWarningPattern(0, 255, 0, 3);
    //=================================
    // QTR Test 
    //=================================
    // The onboard LED will glow blue. 
    // The serial monitor will show the individual QTR LED decay times. 
    // To complete the test individually cover each LED of the QTR with your finger and observe the serial monitor. 
    // The onboard LED will blink blue three times. 
    // The completion message is shown 

    Serial.print("Denken sie mit dem Finger nacheinander die einzelnen QTR LED's ab bis das Passed Array keine 0 mehr enthält, Sobald die LED Blau leuchtet können sie anfangen.\n");
    setOnboardLedColor(0, 0, 255); //Blue


    bool passed[QTR_SENSOR_COUNT] = {false};
    bool ready = false;

    unsigned long startTime = millis();
    
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.read(sensorValues);

    while (millis() - startTime < 200000)
    {
      qtr.read(sensorValues);
  
      while(!ready)
      {
        int check = 0;
        for ( int i = 0; i < QTR_SENSOR_COUNT; ++i)
        {
          if (sensorValues[i] < 400)
          {
            ++check; 
          } 
        }
        
        if (check > 2)
        {
          Serial.print("Stellen sie sicher, dass der QTR Sensor nicht in Richtung einer Oberfläche zeigt. \n");
          
          delay(5000);
          qtr.read(sensorValues);
        }
        else
        {
          ready = true;
        }
      }
      // Check the treshold 
      for (int i = 0; i < QTR_SENSOR_COUNT; i++)
      {
        if (sensorValues[i] < 150)
        {
          passed[i] = true;
        }
      }
      printSensorValues(qtr);
      Serial.print("Passed: ");

      for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
      {
        Serial.print(passed[i]);
        Serial.print(" ");
      }
      Serial.println();

      // Check if all have passed. 
      bool allPassed = true;
      for (int i = 0; i < QTR_SENSOR_COUNT; i++)
      {
        if (!passed[i])
        {
        allPassed = false;
        break;
        }
      }

      if (allPassed)
      {
        break;
      }
      delay(100);
    }

    clearOnboardLed();
    blinkWarningPattern(0, 0, 255, 3);

    Serial.print("Damit ist der Mechatronik Test abgeschlossen. :)\n");
    Serial.print("Sollte eine der Komponenten nicht funktioniert, dann besuchen sie die Mechatronik Sprechstunde, wo ihnen die defekte Komponenten gegebenenfalls ersetzt wird\n");
    
    test = false;
  }

}
