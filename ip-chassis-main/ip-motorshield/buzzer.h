#pragma once
#include <stdint.h>

// Passive piezo buzzer on BUZZER_PIN (pinout.h). Sounds play in their own FreeRTOS task,
// so playSound() returns immediately and never blocks the line following or a checkpoint action.
// A new sound interrupts the one that is currently playing.

enum class Sound : uint8_t
{
    Startup,      // rising chime: robot is ready
    Start,        // "go": robot starts driving
    Stop,         // falling two-tone: robot stopped
    Checkpoint,   // short blip: cross line counted
    Distribution, // package handed over at the distribution center
    RevolverLoad, // ratchet clicks: revolver turning
    Shoot,        // "pew": package shot
    Turn,         // siren: 180 degree turn
    Finish,       // victory fanfare: run complete
    Error,        // low buzz: action failed
    BatteryLow,   // two low beeps
    Count
};

void initBuzzer();                         // call once in setup(), after shield.begin()
void playSound(Sound sound);               // non-blocking; ignored while enableBuzzer is false
void stopSound();                          // silences the buzzer immediately
bool soundFromName(const char *name, Sound &out); // "shoot" -> Sound::Shoot, used by the web interface
const char *soundName(Sound sound);
