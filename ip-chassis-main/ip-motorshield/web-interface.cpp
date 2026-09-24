#include "web-interface.h"

#include "config.h"
#include "onboard-led.h"
#include "shared-globals.h"
#include "web-page.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_wifi.h>
#include <cstring>
#include <cstdio>

namespace
{
WebServer webServer(80);
WebSocketsServer webSocketServer(81);
PIDController *activePid = nullptr;
Motorshield *activeShield = nullptr;
int lastApStationCount = -1;
uint8_t websocketClientCount = 0;
constexpr uint8_t apChannels[] = {1, 6, 11};

// Caps the task sleep so HTTP and WebSocket traffic stays responsive at low telemetry rates.
constexpr uint32_t serviceMaxSleepMs = 4;

// Kept independent of telemetryHzTarget: one sampling pass blocks on I2C for
// several milliseconds, so pacing it with telemetry would stall the task.
constexpr uint32_t monitoringSampleHz = 10;

uint32_t nextTelemetryAtUs = 0;
uint32_t nextMonitorAtUs = 0;
bool pacingInitialized = false;

uint32_t telemetrySendCounter = 0;
uint32_t rateWindowStartMs = 0;
uint32_t execPeakUsInWindow = 0;

char accessPointSsid[32] = {0};

struct __attribute__((packed)) TelemetryBin
{
    uint32_t magic;
    uint16_t version;
    uint16_t total_len;

    uint16_t qtr0;
    uint16_t qtr1;
    uint16_t qtr2;
    uint16_t qtr3;
    uint16_t qtr4;

    int32_t sensorError;
    int32_t leftSpeed;
    int32_t rightSpeed;

    // Supply/current monitoring; NAN while monitoring is disabled.
    float batteryVoltageV;
    float servoRailVoltageV;
    float motorCurrentsA[2];

    float P;
    float I;
    float D;

    int32_t motorSpeed;
    int32_t checkpointCounter;
    int32_t lastCheckpointTimeMs;

    int32_t driveLoopHzTarget;
    int32_t telemetryHzTarget;
    float driveLoopHz_measured;
    float driveLoopExecUs_measured;
    float telemetryHz_measured;
    float telemetryExecUs_peak;
    uint32_t timestamp_ms;
};

void buildAccessPointSsid()
{
    uint8_t mac[6] = {0};

    // Preferred on ESP32 in AP mode.
    WiFi.softAPmacAddress(mac);

    const bool apMacLooksUnset = (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 && mac[3] == 0 && mac[4] == 0 && mac[5] == 0);
    if (apMacLooksUnset)
    {
        // Fallback to generic MAC API.
        WiFi.macAddress(mac);
    }

    const bool macStillUnset = (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 && mac[3] == 0 && mac[4] == 0 && mac[5] == 0);
    if (macStillUnset)
    {
        // Bootstrap AP once so MAC registers become valid on boards that require it.
        constexpr char tempApSsid[] = "IP_temp_dummy_network";
        constexpr char tempApPassword[] = "IP_temp_dummy_network";
        if (WiFi.softAP(tempApSsid, tempApPassword))
        {
            delay(50);
            WiFi.softAPmacAddress(mac);
            WiFi.softAPdisconnect(true);
            delay(50);
        }
    }

    Serial.print("WiFi MAC Address: ");
    for (int i = 0; i < 6; ++i)
    {
        Serial.print(mac[i], HEX);
        if (i < 5)
        {
            Serial.print(":");
        }
    }
    Serial.println();

    if (accessPointTeamName[0] != '\0')
    {
        snprintf(accessPointSsid,
                 sizeof(accessPointSsid),
                 "%s-%s-%02X%02X%02X",
                 accessPointNamePrefix,
                 accessPointTeamName,
                 mac[3],
                 mac[4],
                 mac[5]);
    }
    else
    {
        snprintf(accessPointSsid,
                 sizeof(accessPointSsid),
                 "%s-%02X%02X%02X",
                 accessPointNamePrefix,
                 mac[3],
                 mac[4],
                 mac[5]);
    }
}

uint8_t selectAccessPointChannel()
{
    uint8_t mac[6] = {0};
    if (WiFi.softAPmacAddress(mac) == nullptr)
    {
        WiFi.macAddress(mac);
    }

    const uint8_t channelIndex = mac[5] % (sizeof(apChannels) / sizeof(apChannels[0]));
    return apChannels[channelIndex];
}

void fillParamsMessage(char *buffer, size_t bufferSize)
{
    snprintf(buffer, bufferSize, "params=%.5f,%.5f,%.5f,%d,%d,%d,%d,%d,%d,%d,%d",
             pidControlKp,
             pidControlKi,
             pidControlKd,
             defaultMotorSpeed,
             driveLoopHzTarget,
             telemetryHzTarget,
             enableMotorDeadzoneCompensation ? 1 : 0,
             minPWMtoMoveMotor,
             stopAtCheckpoints ? 1 : 0,
             brakeWhenStopped ? 1 : 0,
             enableSupplyAndM1M2CurrentMonitoring ? 1 : 0);
}

static inline TelemetryBin makeTelemetryBin()
{
    TelemetryBin telemetry{};
    telemetry.magic = 0x314D4C54;
    telemetry.version = 3;
    telemetry.total_len = sizeof(TelemetryBin);

    telemetry.qtr0 = qtrSensorValues[0];
    telemetry.qtr1 = qtrSensorValues[1];
    telemetry.qtr2 = qtrSensorValues[2];
    telemetry.qtr3 = qtrSensorValues[3];
    telemetry.qtr4 = qtrSensorValues[4];

    telemetry.sensorError = pidError;
    telemetry.leftSpeed = leftMotorSpeed;
    telemetry.rightSpeed = rightMotorSpeed;

    telemetry.batteryVoltageV = batteryVoltageV;
    telemetry.servoRailVoltageV = servoRailVoltageV;
    for (uint8_t m = 0; m < 2; ++m)
    {
        telemetry.motorCurrentsA[m] = motorCurrentsA[m];
    }

    telemetry.P = pidControlKp;
    telemetry.I = pidControlKi;
    telemetry.D = pidControlKd;

    telemetry.motorSpeed = defaultMotorSpeed;
    telemetry.checkpointCounter = checkpointCounter;
    telemetry.lastCheckpointTimeMs = lastCheckpointTimeMs;

    telemetry.driveLoopHzTarget = driveLoopHzTarget;
    telemetry.telemetryHzTarget = telemetryHzTarget;
    telemetry.driveLoopHz_measured = driveLoopHzMeasured;
    telemetry.driveLoopExecUs_measured = driveLoopExecUsMeasured;
    telemetry.telemetryHz_measured = telemetryHzMeasured;
    telemetry.telemetryExecUs_peak = telemetryExecUsPeak;
    telemetry.timestamp_ms = millis();

    return telemetry;
}

void sendTelemetryBin()
{
    TelemetryBin telemetry = makeTelemetryBin();
    webSocketServer.broadcastBIN(reinterpret_cast<const uint8_t *>(&telemetry), sizeof(telemetry));
}

void sendParamsToClient(uint8_t clientNum)
{
    char paramsMessage[160];
    fillParamsMessage(paramsMessage, sizeof(paramsMessage));
    webSocketServer.sendTXT(clientNum, paramsMessage);
}

void broadcastParams()
{
    char paramsMessage[160];
    fillParamsMessage(paramsMessage, sizeof(paramsMessage));
    webSocketServer.broadcastTXT(paramsMessage);
}

void applyParamsFromCsv(const char *payload)
{
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    int motorSpeed = 0;
    int driveLoopHzIn = 0;
    int telemetryHzIn = 0;
    int deadzoneEnabled = 0;
    int minPwm = 0;
    int stopAtCheckpoint = 0;
    int brakeAtStop = 0;
    int monitoringEnabled = 0;

    if (sscanf(payload,
               "params=%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d",
               &p,
               &i,
               &d,
               &motorSpeed,
               &driveLoopHzIn,
               &telemetryHzIn,
               &deadzoneEnabled,
               &minPwm,
               &stopAtCheckpoint,
               &brakeAtStop,
               &monitoringEnabled) != 11)
    {
        Serial.print("Invalid params payload: ");
        Serial.println(payload);
        return;
    }

    pidControlKp = p;
    pidControlKi = i;
    pidControlKd = d;
    defaultMotorSpeed = constrain(motorSpeed, 0, PWM_MAX_VALUE);
    driveLoopHzTarget = constrain(driveLoopHzIn, 1, 500);
    telemetryHzTarget = constrain(telemetryHzIn, 1, 200);
    enableMotorDeadzoneCompensation = deadzoneEnabled != 0;
    minPWMtoMoveMotor = constrain(minPwm, 0, PWM_MAX_VALUE);
    stopAtCheckpoints = stopAtCheckpoint != 0;
    brakeWhenStopped = brakeAtStop != 0;
    enableSupplyAndM1M2CurrentMonitoring = monitoringEnabled != 0;

    if (activePid != nullptr)
    {
        activePid->kp = pidControlKp;
        activePid->ki = pidControlKi;
        activePid->kd = pidControlKd;
        activePid->integral = 0;
        activePid->prevError = 0;
    }

    Serial.printf("Params updated: P=%.5f I=%.5f D=%.5f motor=%d driveHzTarget=%d telemetryHzTarget=%d deadzone=%d minPWM=%d stopAtCheckpoint=%d brakeWhenStopped=%d monitoring=%d\n",
                  pidControlKp,
                  pidControlKi,
                  pidControlKd,
                  defaultMotorSpeed,
                  driveLoopHzTarget,
                  telemetryHzTarget,
                  enableMotorDeadzoneCompensation ? 1 : 0,
                  minPWMtoMoveMotor,
                  stopAtCheckpoints ? 1 : 0,
                  brakeWhenStopped ? 1 : 0,
                  enableSupplyAndM1M2CurrentMonitoring ? 1 : 0);
}

void applyWebCommand(const char *payload, uint8_t clientNum)
{
    if (strncmp(payload, "command=start", 13) == 0)
    {
        robotMotionEnabled = true;
        webSocketServer.sendTXT(clientNum, "status=moving");
        return;
    }

    if (strncmp(payload, "command=stop", 12) == 0)
    {
        robotMotionEnabled = false;
        webSocketServer.sendTXT(clientNum, "status=stopped");
        return;
    }

    if (strncmp(payload, "command=get_params", 18) == 0)
    {
        sendParamsToClient(clientNum);
        return;
    }
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_CONNECTED:
        ++websocketClientCount;
        setHeartbeatWebsocketClientActive(websocketClientCount > 0);
        Serial.printf("WebSocket client connected: #%u (active: %u)\n",
                      static_cast<unsigned>(clientNum),
                      static_cast<unsigned>(websocketClientCount));
        break;
    case WStype_DISCONNECTED:
        if (websocketClientCount > 0)
        {
            --websocketClientCount;
        }
        setHeartbeatWebsocketClientActive(websocketClientCount > 0);
        Serial.printf("WebSocket client disconnected: #%u (active: %u)\n",
                      static_cast<unsigned>(clientNum),
                      static_cast<unsigned>(websocketClientCount));
        break;
    case WStype_TEXT:
        if (payload == nullptr || length == 0)
        {
            return;
        }
        if (strncmp(reinterpret_cast<const char *>(payload), "params=", 7) == 0)
        {
            char text[160];
            const size_t copyLength = min(length, sizeof(text) - 1);
            memcpy(text, payload, copyLength);
            text[copyLength] = '\0';
            Serial.print("Received params message: ");
            Serial.println(text);
            applyParamsFromCsv(text);
            broadcastParams();
        }
        else if (strncmp(reinterpret_cast<const char *>(payload), "command=", 8) == 0)
        {
            char text[64];
            const size_t copyLength = min(length, sizeof(text) - 1);
            memcpy(text, payload, copyLength);
            text[copyLength] = '\0';
            applyWebCommand(text, clientNum);
        }
        break;
    default:
        break;
    }
}

void handleHome()
{
    webServer.send_P(200, "text/html", htmlPage);
}

// Resynchronises to now when more than a whole period was missed, instead of
// replaying the backlog.
void advanceDeadline(uint32_t &deadlineUs, uint32_t periodUs, uint32_t nowUs)
{
    deadlineUs += periodUs;
    if (static_cast<int32_t>(nowUs - deadlineUs) >= 0)
    {
        deadlineUs = nowUs + periodUs;
    }
}

// Samples both supply rails and the averaged M1/M2 currents while monitoring
// is enabled. Blocks on I2C for several milliseconds.
void sampleSupplyAndM1M2CurrentMonitoring()
{
    if (enableSupplyAndM1M2CurrentMonitoring && activeShield != nullptr)
    {
        batteryVoltageV = activeShield->batteryVoltage();
        servoRailVoltageV = activeShield->servoRailVoltage();
        for (uint8_t m = 0; m < 2; ++m)
        {
            motorCurrentsA[m] = activeShield->motor(m + 1).readCurrentAmps();
        }
    }
    else
    {
        batteryVoltageV = NAN;
        servoRailVoltageV = NAN;
        for (uint8_t m = 0; m < 2; ++m)
        {
            motorCurrentsA[m] = NAN;
        }
    }
}
}

void initWebInterface(PIDController &pid, Motorshield *shield)
{
    activePid = &pid;
    activeShield = shield;

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    buildAccessPointSsid();
    const uint8_t accessPointChannel = selectAccessPointChannel();
    if (!WiFi.softAP(accessPointSsid, accessPointPassword, accessPointChannel, 0, 1))
    {
        Serial.println("Failed to start WiFi access point");
        while (true)
        {
            delay(1000);
        }
    }

    if (!WiFi.setTxPower(WIFI_POWER_5dBm))
    {
        Serial.println("Failed to set WiFi TX power");
    }
    WiFi.softAPbandwidth(WIFI_BW_HT20);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    wifi_config_t apConfig;
    memset(&apConfig, 0, sizeof(apConfig));
    if (esp_wifi_get_config(WIFI_IF_AP, &apConfig) == ESP_OK)
    {
        apConfig.ap.beacon_interval = 1000;
        esp_wifi_set_config(WIFI_IF_AP, &apConfig);
    }

    Serial.print("WiFi AP started: ");
    Serial.println(accessPointSsid);
    Serial.print("AP channel: ");
    Serial.println(accessPointChannel);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    webServer.on("/", HTTP_GET, handleHome);
    webServer.begin();

    webSocketServer.begin();
    webSocketServer.onEvent(onWebSocketEvent);
    webSocketServer.enableHeartbeat(15000, 3000, 2);
    pacingInitialized = false;
}

uint32_t updateWebInterface()
{
    const uint32_t startedAtUs = micros();
    webServer.handleClient();
    webSocketServer.loop();

    const int currentApStationCount = WiFi.softAPgetStationNum();
    if (currentApStationCount != lastApStationCount)
    {
        if (lastApStationCount >= 0)
        {
            if (currentApStationCount > lastApStationCount)
            {
                Serial.printf("AP station connected (total: %d)\n", currentApStationCount);
            }
            else
            {
                Serial.printf("AP station disconnected (total: %d)\n", currentApStationCount);
            }
        }
        lastApStationCount = currentApStationCount;
    }

    const uint32_t telemetryPeriodUs = 1000000UL / static_cast<uint32_t>(max(telemetryHzTarget, 1));
    constexpr uint32_t monitorPeriodUs = 1000000UL / monitoringSampleHz;

    uint32_t nowUs = micros();
    if (!pacingInitialized)
    {
        nextTelemetryAtUs = nowUs;
        nextMonitorAtUs = nowUs;
        pacingInitialized = true;
    }

    if (static_cast<int32_t>(nowUs - nextMonitorAtUs) >= 0)
    {
        sampleSupplyAndM1M2CurrentMonitoring();
        nowUs = micros();
        advanceDeadline(nextMonitorAtUs, monitorPeriodUs, nowUs);
    }

    // The deadline advances even with no client connected, so it cannot fall arbitrarily behind.
    if (static_cast<int32_t>(nowUs - nextTelemetryAtUs) >= 0)
    {
        if (websocketClientCount > 0)
        {
            sendTelemetryBin();
            ++telemetrySendCounter;
            nowUs = micros();
        }
        advanceDeadline(nextTelemetryAtUs, telemetryPeriodUs, nowUs);
    }

    const uint32_t execUs = micros() - startedAtUs;
    if (execUs > execPeakUsInWindow)
    {
        execPeakUsInWindow = execUs;
    }

    const uint32_t nowMs = millis();
    if (rateWindowStartMs == 0)
    {
        rateWindowStartMs = nowMs;
    }
    if (nowMs - rateWindowStartMs >= 1000)
    {
        telemetryHzMeasured = (1000.0f * static_cast<float>(telemetrySendCounter)) / static_cast<float>(nowMs - rateWindowStartMs);
        telemetryExecUsPeak = static_cast<float>(execPeakUsInWindow);
        telemetrySendCounter = 0;
        execPeakUsInWindow = 0;
        rateWindowStartMs = nowMs;
    }

    const int32_t untilTelemetryUs = static_cast<int32_t>(nextTelemetryAtUs - micros());
    if (untilTelemetryUs <= 0)
    {
        return 0;
    }
    const uint32_t sleepMs = static_cast<uint32_t>(untilTelemetryUs) / 1000UL;
    return sleepMs < serviceMaxSleepMs ? sleepMs : serviceMaxSleepMs;
}
