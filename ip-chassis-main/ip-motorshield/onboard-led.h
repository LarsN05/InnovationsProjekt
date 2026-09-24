#pragma once

#include <stdint.h>

void initOnboardLed();
void setOnboardLedColor(uint8_t red, uint8_t green, uint8_t blue);
void clearOnboardLed();
void blinkPeriodically();
void blinkWarningPattern(uint8_t red, uint8_t green, uint8_t blue, uint8_t count);
void setHeartbeatWebsocketClientActive(bool hasActiveWebsocketClient);
void setHeartbeatStandaloneActive();
