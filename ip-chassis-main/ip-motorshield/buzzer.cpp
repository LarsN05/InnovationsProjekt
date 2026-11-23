#include "buzzer.h"

#include "config.h"
#include "pinout.h"

#include <Arduino.h>
#include <esp32-hal-periman.h>
#include <cstring>

namespace
{

// One segment of a sound: the frequency slides from startHz to endHz over ms (equal = plain note, 0 = rest).
struct Tone
{
    uint16_t startHz;
    uint16_t endHz;
    uint16_t ms;
};

#define NOTE(hz, ms) {hz, hz, ms}
#define SLIDE(fromHz, toHz, ms) {fromHz, toHz, ms}
#define REST(ms) {0, 0, ms}

// Note frequencies (Hz). A small piezo is loudest around its resonance (ca. 2-4 kHz),
// so all sounds use the 7th octave; lower notes would be much quieter.
constexpr uint16_t C7 = 2093, D7 = 2349, E7 = 2637, G7 = 3136, A7 = 3520, C8 = 4186;

const Tone startupTones[] = {NOTE(C7, 90), NOTE(E7, 90), NOTE(G7, 90), NOTE(C8, 220)};
const Tone startTones[] = {NOTE(G7, 80), REST(40), NOTE(C8, 180)};
const Tone stopTones[] = {NOTE(C8, 120), NOTE(G7, 120), NOTE(C7, 250)};
const Tone checkpointTones[] = {NOTE(A7, 60)};
const Tone distributionTones[] = {SLIDE(4200, 2000, 350), REST(60), NOTE(C7, 80), NOTE(G7, 140)};
const Tone revolverTones[] = {NOTE(4000, 20), REST(70), NOTE(4000, 20), REST(70), NOTE(4000, 20), REST(70),
                              NOTE(4000, 20), REST(70), NOTE(3500, 40)};
const Tone shootTones[] = {SLIDE(4500, 1800, 220)};
const Tone turnTones[] = {SLIDE(2200, 3800, 250), SLIDE(3800, 2200, 250), SLIDE(2200, 3800, 250), SLIDE(3800, 2200, 250)};
const Tone finishTones[] = {NOTE(C7, 110), NOTE(C7, 110), NOTE(C7, 110), NOTE(C7, 330), NOTE(1760, 330),
                            NOTE(D7, 330), NOTE(C7, 160), REST(80), NOTE(D7, 110), NOTE(C7, 500)};
const Tone errorTones[] = {NOTE(1500, 250), REST(80), NOTE(1500, 250), REST(80), NOTE(1200, 500)};
const Tone batteryLowTones[] = {NOTE(2000, 150), REST(100), NOTE(1600, 300)};

struct SoundDef
{
    const char *name;
    const Tone *tones;
    uint8_t count;
};

#define SOUND(name, tones) {name, tones, sizeof(tones) / sizeof(tones[0])}

// Same order as enum class Sound
const SoundDef sounds[] = {
    SOUND("startup", startupTones),
    SOUND("start", startTones),
    SOUND("stop", stopTones),
    SOUND("checkpoint", checkpointTones),
    SOUND("distribution", distributionTones),
    SOUND("revolver", revolverTones),
    SOUND("shoot", shootTones),
    SOUND("turn", turnTones),
    SOUND("finish", finishTones),
    SOUND("error", errorTones),
    SOUND("battery", batteryLowTones),
};
static_assert(sizeof(sounds) / sizeof(sounds[0]) == static_cast<size_t>(Sound::Count), "sounds[] must match enum Sound");

constexpr uint32_t stepMs = 5;      // frequency update interval for slides
constexpr uint8_t stopRequest = 0xFF;

QueueHandle_t soundQueue = nullptr;
bool buzzerReady = false;

bool pushPull = false;

void setTone(uint32_t hz)
{
    ledcWriteTone(BUZZER_PIN, hz); // 0 = silent
    if (pushPull)
    {
        // Second leg gets the inverted signal while a tone plays (double voltage swing = louder),
        // and stays low while silent, so no DC voltage sits on the buzzer.
        ledcOutputInvert(BUZZER_PIN_2, hz != 0);
    }
}

// Waits stepMs; returns true if a new request arrived (current sound is interrupted).
bool waitStep()
{
    vTaskDelay(pdMS_TO_TICKS(stepMs));
    return uxQueueMessagesWaiting(soundQueue) > 0;
}

void play(const SoundDef &sound)
{
    for (uint8_t i = 0; i < sound.count; ++i)
    {
        const Tone &t = sound.tones[i];
        const uint32_t steps = max<uint32_t>(1, t.ms / stepMs);
        uint32_t lastHz = UINT32_MAX;
        for (uint32_t s = 0; s < steps; ++s)
        {
            const uint32_t hz = t.startHz + (int32_t)(t.endHz - t.startHz) * (int32_t)s / (int32_t)steps;
            if (hz != lastHz)
            {
                setTone(hz);
                lastHz = hz;
            }
            if (waitStep())
            {
                setTone(0);
                return;
            }
        }
    }
    setTone(0);
}

void buzzerTask(void *)
{
    uint8_t request = 0;
    for (;;)
    {
        if (xQueueReceive(soundQueue, &request, portMAX_DELAY) == pdTRUE)
        {
            if (request < static_cast<uint8_t>(Sound::Count))
            {
                play(sounds[request]);
            }
            else
            {
                setTone(0);
            }
        }
    }
}

void request(uint8_t value)
{
    if (!buzzerReady)
        return;
    xQueueOverwrite(soundQueue, &value); // the newest request wins
}

} // namespace

void initBuzzer()
{
    if (!enableBuzzer)
        return;
    // 2 kHz / 10 bit: a different setting than the drive motors (1526 Hz / 12 bit),
    // so the buzzer gets its own LEDC timer and never changes the motor PWM frequency.
    if (!ledcAttach(BUZZER_PIN, 2000, 10))
    {
        Serial.println("Buzzer: LEDC attach failed");
        return;
    }
    if (enableBuzzerPushPull)
    {
        // Attach the second pin to the same LEDC channel, so both legs switch exactly together.
        const auto *bus = static_cast<ledc_channel_handle_t *>(perimanGetPinBus(BUZZER_PIN, ESP32_BUS_TYPE_LEDC));
        pushPull = bus != nullptr && ledcAttachChannel(BUZZER_PIN_2, 2000, 10, bus->channel);
        if (!pushPull)
        {
            Serial.println("Buzzer: push-pull setup failed, using one pin");
        }
    }
    setTone(0);
    soundQueue = xQueueCreate(1, sizeof(uint8_t));
    xTaskCreatePinnedToCore(buzzerTask, "buzzer", 2048, nullptr, 1, nullptr, 0);
    buzzerReady = true;
}

void playSound(Sound sound)
{
    request(static_cast<uint8_t>(sound));
}

void stopSound()
{
    request(stopRequest);
}

bool soundFromName(const char *name, Sound &out)
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sound::Count); ++i)
    {
        if (strcmp(name, sounds[i].name) == 0)
        {
            out = static_cast<Sound>(i);
            return true;
        }
    }
    return false;
}

const char *soundName(Sound sound)
{
    const uint8_t i = static_cast<uint8_t>(sound);
    return i < static_cast<uint8_t>(Sound::Count) ? sounds[i].name : "?";
}
