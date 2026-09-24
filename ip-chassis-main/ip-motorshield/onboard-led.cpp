#include "onboard-led.h"

#include "pinout.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>


Adafruit_NeoPixel rgbLed(1, ESP32_RGB_LED_DATA_PIN, NEO_RGB + NEO_KHZ800);

unsigned long lastFlashTime = 0;

enum class HeartbeatMode
{
    IdleGreen,
    WebsocketOrange,
    StandaloneCyan,
};

HeartbeatMode heartbeatMode = HeartbeatMode::IdleGreen;

void applyHeartbeatOnColor()
{
    switch (heartbeatMode)
    {
    case HeartbeatMode::WebsocketOrange:
        setOnboardLedColor(255, 80, 0);
        break;
    case HeartbeatMode::StandaloneCyan:
        setOnboardLedColor(0, 200, 200);
        break;
    case HeartbeatMode::IdleGreen:
    default:
        setOnboardLedColor(0, 128, 0);
        break;
    }
}

void initOnboardLed()
{
    rgbLed.begin();
}

void setOnboardLedColor(uint8_t red, uint8_t green, uint8_t blue)
{
    rgbLed.setPixelColor(0, rgbLed.Color(red, green, blue));
    rgbLed.show();
}

void clearOnboardLed()
{
    rgbLed.clear();
    rgbLed.show();
}

void blinkPeriodically()
{
    const unsigned long currentTime = millis();
    if (currentTime - lastFlashTime >= 1000)
    {
        lastFlashTime = currentTime;
        applyHeartbeatOnColor();
    }
    else if (currentTime - lastFlashTime >= 100)
    {
        clearOnboardLed();
    }
}

void blinkWarningPattern(uint8_t red, uint8_t green, uint8_t blue, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i)
    {
        setOnboardLedColor(red, green, blue);
        delay(160);
        clearOnboardLed();
        delay(160);
    }
}

void setHeartbeatWebsocketClientActive(bool hasActiveWebsocketClient)
{
    heartbeatMode = hasActiveWebsocketClient ? HeartbeatMode::WebsocketOrange : HeartbeatMode::IdleGreen;
}

void setHeartbeatStandaloneActive()
{
    heartbeatMode = HeartbeatMode::StandaloneCyan;
}