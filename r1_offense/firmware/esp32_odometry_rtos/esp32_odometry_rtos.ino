/**
 * esp32_odometry_rtos.ino — R1 Offense: Dual-Core Odometry + WebSocket Server
 * =============================================================================
 * Platform : ESP32 DevKit
 * Framework: Arduino + FreeRTOS (built into ESP-IDF)
 *
 * Overview
 * --------
 * This firmware runs two independent FreeRTOS tasks pinned to separate cores:
 *
 *   Core 0 — Task_EncoderRead (100 Hz)
 *       Samples three quadrature encoders (X-upper, X-lower, Y) and
 *       integrates world-frame position. Heading is derived from the
 *       differential between the two X-axis encoders (a passive odometry
 *       wheel arrangement). Position and heading are streamed to the
 *       Arduino Due over USART using the EasyTransfer library.
 *
 *   Core 1 — Task_WebSocket (10 Hz cleanup)
 *       Hosts an async WebSocket server over WiFi. Receives JSON messages
 *       from a phone/tablet slider UI and updates the pneumatic dribbler
 *       position setpoint. This allows the drive team to adjust the
 *       dribbler height mid-match without a dedicated button channel.
 *
 * Encoder geometry
 * ----------------
 *   encoderXU  Upper X-axis encoder (left of centre)
 *   encoderXD  Lower X-axis encoder (right of centre)
 *   encoderY   Y-axis encoder (perpendicular to X)
 *
 *   Heading is estimated from the differential displacement of XU and XD:
 *       Δangle = (Δxd - Δxu) * dist_per_pulse / (2 * enc_radius)
 *
 *   World-frame position is obtained by rotating body-frame deltas:
 *       Xa = Xb * cos(angle) - Yb * sin(angle)
 *       Ya = Xb * sin(angle) + Yb * cos(angle)
 *
 * USART output (to Arduino Due via EasyTransfer)
 * -----------------------------------------------
 *   Struct SEND_READING {
 *     double currentDist_x;   // world-frame X, cm
 *     double currentDist_y;   // world-frame Y, cm
 *     double angle;           // heading, radians
 *     double slider;          // dribbler position setpoint (0–100)
 *   }
 *
 * WebSocket input (from phone/tablet UI)
 * ----------------------------------------
 *   JSON: { "position": <float> }
 *   Non-zero values update the dribbler position setpoint.
 */

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "EasyTransfer.h"

// ---------------------------------------------------------------------------
// WiFi credentials — replace before deployment
// ---------------------------------------------------------------------------
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// ---------------------------------------------------------------------------
// Odometry constants
// ---------------------------------------------------------------------------
const double WHEEL_CIRCUMFERENCE_CM  = 18.84;   // cm — passive odometry wheel
const double ENCODER_RADIUS_CM       = 30.3;    // cm — half-distance between XU and XD
const double PULSES_PER_REVOLUTION   = 10000.0;
const double DIST_PER_PULSE          = WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REVOLUTION;

// ---------------------------------------------------------------------------
// FreeRTOS task parameters
// ---------------------------------------------------------------------------
#define ENCODER_TASK_STACK  4096
#define WEBSOCKET_TASK_STACK 4096
#define ENCODER_TASK_PERIOD_MS   10   //  100 Hz
#define WEBSOCKET_CLEANUP_MS    100   //   10 Hz

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
ESP32Encoder encoderXU;   // Upper X-axis
ESP32Encoder encoderXD;   // Lower X-axis
ESP32Encoder encoderY;    // Y-axis

AsyncWebServer  webServer(80);
AsyncWebSocket  webSocket("/ws");
EasyTransfer    ET;

// ---------------------------------------------------------------------------
// Shared state (written by Task_WebSocket, read by Task_EncoderRead)
// Use volatile to prevent compiler optimisation across task boundaries.
// ---------------------------------------------------------------------------
volatile double g_sliderPosition = 0.0;

// ---------------------------------------------------------------------------
// USART output struct
// ---------------------------------------------------------------------------
struct SEND_READING {
    double currentDist_x;
    double currentDist_y;
    double angle;
    double slider;
};
SEND_READING odometry = {0.0, 0.0, 0.0, 0.0};

// ---------------------------------------------------------------------------
// Odometry state (Task_EncoderRead only — no shared-state concern)
// ---------------------------------------------------------------------------
double prevTicksXU = 0.0;
double prevTicksXD = 0.0;
double prevTicksX  = 0.0;
double prevTicksY  = 0.0;


// ---------------------------------------------------------------------------
// Task: Core 0 — Encoder sampling and odometry integration
// ---------------------------------------------------------------------------
void Task_EncoderRead(void* pvParameters) {
    for (;;) {
        // Read raw counts
        double ticksXU = (double)encoderXU.getCount();
        double ticksXD = (double)encoderXD.getCount();
        double ticksX  = (ticksXU + ticksXD) / 2.0;   // averaged X
        double ticksY  = (double)encoderY.getCount();

        // Incremental deltas since last sample
        double deltaXU = ticksXU - prevTicksXU;
        double deltaXD = ticksXD - prevTicksXD;
        double deltaXb = ticksX  - prevTicksX;    // body-frame X
        double deltaYb = ticksY  - prevTicksY;    // body-frame Y

        // Heading update from differential X encoders
        // Positive deltaXD - deltaXU → counter-clockwise rotation
        odometry.angle += ((deltaXD - deltaXU) * DIST_PER_PULSE) / (2.0 * ENCODER_RADIUS_CM);

        // Rotate body-frame displacement into world frame
        double Xa = deltaXb * cos(odometry.angle) - deltaYb * sin(odometry.angle);
        double Ya = deltaXb * sin(odometry.angle) + deltaYb * cos(odometry.angle);

        odometry.currentDist_x += Xa * DIST_PER_PULSE;
        odometry.currentDist_y += Ya * DIST_PER_PULSE;
        odometry.slider         = g_sliderPosition;

        // Stream odometry to Arduino Due
        ET.sendData();

        // Advance previous-tick accumulators
        prevTicksXU = ticksXU;
        prevTicksXD = ticksXD;
        prevTicksX  = ticksX;
        prevTicksY  = ticksY;

        vTaskDelay(pdMS_TO_TICKS(ENCODER_TASK_PERIOD_MS));
    }
}


// ---------------------------------------------------------------------------
// Task: Core 1 — WebSocket housekeeping
// ---------------------------------------------------------------------------
void Task_WebSocket(void* pvParameters) {
    for (;;) {
        webSocket.cleanupClients();
        vTaskDelay(pdMS_TO_TICKS(WEBSOCKET_CLEANUP_MS));
    }
}


// ---------------------------------------------------------------------------
// WebSocket event handler
// ---------------------------------------------------------------------------
void onWebSocketEvent(AsyncWebSocket* server,
                      AsyncWebSocketClient* client,
                      AwsEventType type,
                      void* arg,
                      uint8_t* data,
                      size_t length)
{
    if (type != WS_EVT_DATA) return;

    // Null-terminate the incoming message
    String msg;
    for (size_t i = 0; i < length; i++) msg += (char)data[i];

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg) != DeserializationError::Ok) return;

    if (doc.containsKey("position")) {
        double newPos = doc["position"].as<double>();
        if (newPos != 0.0) {
            g_sliderPosition = newPos;
        }
    }
}


// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.println("\nConnected: " + WiFi.localIP().toString());

    // WebSocket server
    webSocket.onEvent(onWebSocketEvent);
    webServer.addHandler(&webSocket);
    webServer.begin();

    // Encoders (full-quadrature = 4× resolution)
    encoderXU.attachFullQuad(12, 14);
    encoderXD.attachFullQuad(17, 16);
    encoderY.attachFullQuad(19, 18);
    encoderXU.clearCount();
    encoderXD.clearCount();
    encoderY.clearCount();

    // EasyTransfer to Arduino Due
    ET.begin(details(odometry), &Serial);

    // Launch FreeRTOS tasks, pinned to separate cores
    xTaskCreatePinnedToCore(Task_EncoderRead, "EncoderRead",
                            ENCODER_TASK_STACK,  nullptr, 1, nullptr, 0);

    xTaskCreatePinnedToCore(Task_WebSocket,   "WebSocket",
                            WEBSOCKET_TASK_STACK, nullptr, 1, nullptr, 1);
}


// loop() is intentionally empty — all work is done in FreeRTOS tasks
void loop() {}
