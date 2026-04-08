/**
 * esp32_odometry_rtos.ino — R1 Offense: Encoder Odometry (FreeRTOS)
 * ===================================================================
 * Platform : ESP32 DevKit
 * Framework: Arduino + FreeRTOS (built into ESP-IDF)
 *
 * Overview
 * --------
 * Runs a FreeRTOS task on Core 0 that reads three quadrature encoders
 * at 100 Hz, integrates world-frame odometry, and streams the result
 * to the Raspberry Pi over USART using EasyTransfer.
 *
 * We used FreeRTOS to pin the encoder task to a dedicated core so that
 * nothing else can preempt it — missing encoder pulses causes accumulated
 * odometry error, so the sampling rate needs to be consistent.
 *
 * Encoder geometry
 * ----------------
 *   encoderXU  Upper X-axis encoder (left of centre)
 *   encoderXD  Lower X-axis encoder (right of centre)
 *   encoderY   Y-axis encoder (perpendicular to X)
 *
 *   Heading is estimated from the differential displacement of XU and XD:
 *       Δangle = (ΔxD - ΔxU) * dist_per_pulse / (2 * enc_radius)
 *
 *   World-frame position is obtained by rotating the body-frame deltas:
 *       Xa = Xb * cos(angle) - Yb * sin(angle)
 *       Ya = Xb * sin(angle) + Yb * cos(angle)
 *
 * USART output (to Raspberry Pi via EasyTransfer)
 * ------------------------------------------------
 *   Struct SEND_READING {
 *     double currentDist_x;   // world-frame X, cm
 *     double currentDist_y;   // world-frame Y, cm
 *     double angle;           // heading, radians
 *   }
 */

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "EasyTransfer.h"

// ---------------------------------------------------------------------------
// Odometry constants
// ---------------------------------------------------------------------------
const double WHEEL_CIRCUMFERENCE_CM = 18.84;    // cm — passive odometry wheel
const double ENCODER_RADIUS_CM      = 30.3;     // cm — half-distance between XU and XD
const double PULSES_PER_REVOLUTION  = 10000.0;
const double DIST_PER_PULSE         = WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REVOLUTION;

// ---------------------------------------------------------------------------
// FreeRTOS task config
// ---------------------------------------------------------------------------
#define ENCODER_TASK_STACK     4096
#define ENCODER_TASK_PERIOD_MS 10     // 100 Hz

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
ESP32Encoder encoderXU;   // Upper X-axis  — pins 12, 14
ESP32Encoder encoderXD;   // Lower X-axis  — pins 17, 16
ESP32Encoder encoderY;    // Y-axis        — pins 19, 18

EasyTransfer ET;

// ---------------------------------------------------------------------------
// USART output struct
// ---------------------------------------------------------------------------
struct SEND_READING {
    double currentDist_x;
    double currentDist_y;
    double angle;
};
SEND_READING odometry = {0.0, 0.0, 0.0};

// ---------------------------------------------------------------------------
// Odometry state (only accessed by Task_EncoderRead)
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
        double deltaXb = ticksX  - prevTicksX;    // body-frame X displacement
        double deltaYb = ticksY  - prevTicksY;    // body-frame Y displacement

        // Heading update from differential X encoders
        // Positive (deltaXD - deltaXU) = counter-clockwise rotation
        odometry.angle += ((deltaXD - deltaXU) * DIST_PER_PULSE) / (2.0 * ENCODER_RADIUS_CM);

        // Rotate body-frame displacement into world frame
        double Xa = deltaXb * cos(odometry.angle) - deltaYb * sin(odometry.angle);
        double Ya = deltaXb * sin(odometry.angle) + deltaYb * cos(odometry.angle);

        odometry.currentDist_x += Xa * DIST_PER_PULSE;
        odometry.currentDist_y += Ya * DIST_PER_PULSE;

        // Stream to Raspberry Pi
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
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Encoders (full-quadrature = 4× resolution)
    encoderXU.attachFullQuad(12, 14);
    encoderXD.attachFullQuad(17, 16);
    encoderY.attachFullQuad(19, 18);
    encoderXU.clearCount();
    encoderXD.clearCount();
    encoderY.clearCount();

    // EasyTransfer to Raspberry Pi over hardware USART
    ET.begin(details(odometry), &Serial);

    // Launch encoder task pinned to Core 0
    xTaskCreatePinnedToCore(
        Task_EncoderRead,
        "EncoderRead",
        ENCODER_TASK_STACK,
        nullptr,
        1,
        nullptr,
        0
    );

    Serial.println("Odometry task started on Core 0");
}


// loop() is intentionally empty — all work is in the FreeRTOS task
void loop() {}
