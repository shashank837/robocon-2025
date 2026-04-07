/**
 * due_omni_controller.ino — R2 Defense Robot: Full Motion Controller
 * ===================================================================
 * Platform: Arduino Due (bare-metal, no RTOS)
 *
 * This firmware runs the complete R2 defense robot on a single Arduino Due.
 * The ESP32 handles PS4 Bluetooth reception and forwards controller state
 * over USART using the EasyTransfer library. The Due translates this into:
 *
 *   - 3-wheel omnidirectional drive with proportional speed scaling
 *   - IMU-based active heading lock (PID angular velocity correction)
 *   - Encoder-gated sliding net deployment mechanism
 *   - Pneumatic net support activation
 *
 * Why bare-metal (no RTOS)?
 * ─────────────────────────
 * R2's control loop has a simple, sequential structure: receive → decide →
 * actuate. There is no concurrent I/O that would benefit from task scheduling.
 * Running bare-metal on the Due gives deterministic, low-latency motor
 * response without the overhead or complexity of an RTOS.
 *
 * 3-Wheel Omni Inverse Kinematics
 * ─────────────────────────────────
 * Wheel layout (120° symmetric):
 *     Motor 1: front-left  (90°)
 *     Motor 2: rear-left   (210°)
 *     Motor 3: rear-right  (330°)
 *
 * Given body-frame commands (vx, vy, ω):
 *     ω₁ = ( 2·vx / 3)                + (ω / 3)
 *     ω₂ = (−vx / 3) + (vy / √3)      + (ω / 3)    [sign matches wiring]
 *     ω₃ = (−vx / 3) − (vy / √3)      + (ω / 3)
 *
 * Speeds are proportionally scaled so the fastest wheel is capped at
 * PWM_MAX without distorting the velocity direction ratio.
 *
 * Sliding Net Mechanism
 * ─────────────────────
 * The net is deployed by a DC motor with a quadrature encoder. When the
 * driver presses Triangle (rising edge detected), the motor runs until
 * NET_TARGET_TICKS encoder counts have elapsed, then stops. This prevents
 * mechanical over-travel without requiring a physical limit switch.
 *
 * EasyTransfer USART
 * ──────────────────
 * Two EasyTransfer channels share the Due's hardware serial ports:
 *   Serial  (USB/Serial0) — PS4 controller data from ESP32
 *   Serial3               — Encoder odometry from secondary ESP32
 */

#include "EasyTransfer.h"
#include <Encoder.h>

// ---------------------------------------------------------------------------
// Motor pins (PWM-capable on Arduino Due)
// ---------------------------------------------------------------------------
#define M1_PWM   8
#define M1_DIR   9
#define M2_PWM  13
#define M2_DIR  12
#define M3_PWM  11
#define M3_DIR  10

// Net mechanism motor
#define NET_PWM  54
#define NET_DIR  55

// Fan / defensive blower
#define FAN_PWM  25
#define FAN_DIR  23

// Pneumatics (two-valve solenoid)
#define PNE_EXTEND  52
#define PNE_RETRACT 53

// Net mechanism quadrature encoder
Encoder netEncoder(56, 57);

// ---------------------------------------------------------------------------
// Control constants
// ---------------------------------------------------------------------------
#define PWM_MAX           255
#define PWM_SCALE_FACTOR  2.5f    // maps raw IK output → [0, 255]
#define JOYSTICK_DEADZONE  10     // ignore stick deflections below this value

// Heading lock PID
#define Kp_HEADING  1.5

// Net encoder travel (ticks to fully deploy)
#define NET_TARGET_TICKS  700L
#define NET_SPEED         150

// ---------------------------------------------------------------------------
// EasyTransfer data structures
// ---------------------------------------------------------------------------
EasyTransfer ET_ps;    // PS4 data from ESP32
EasyTransfer ET_enc;   // Encoder odometry from secondary ESP32

struct PS4_DATA {
    int    x;           // left stick X  (scaled, ±100)
    int    y;           // left stick Y  (scaled, ±100)
    int    r2;          // R2 trigger
    int    l2;          // L2 trigger
    int    r1;          // R1 shoulder button
    bool   up, down, left, right;   // D-pad
    bool   cross, circle, square, triangle;
    double angle;       // IMU heading from ESP32 (degrees)
};

struct ENC_DATA {
    float x;
    float y;
    float angle;
};

PS4_DATA ps4;
ENC_DATA enc;

// ---------------------------------------------------------------------------
// State machine for net mechanism
// ---------------------------------------------------------------------------
bool netRunning        = false;
bool trianglePrevState = false;
long netStartTicks     = 0L;

// ---------------------------------------------------------------------------
// Heading lock state
// ---------------------------------------------------------------------------
double headingTarget = 0.0;
double headingError  = 0.0;
double omega         = 0.0;   // corrective angular velocity


// ---------------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------------

void setMotor(int dirPin, int pwmPin, bool forward, int speed) {
    digitalWrite(dirPin, forward ? HIGH : LOW);
    analogWrite(pwmPin, constrain(speed, 0, PWM_MAX));
}

void stopAllMotors() {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    analogWrite(M3_PWM, 0);
    analogWrite(NET_PWM, 0);
    analogWrite(FAN_PWM, 0);
}

/**
 * Convert a raw IK floating-point wheel speed to a non-negative PWM integer.
 * The sign is handled separately via the direction pin.
 */
int toPWM(float speed) {
    return constrain((int)(abs(speed) * PWM_SCALE_FACTOR), 0, PWM_MAX);
}

/**
 * Drive all three omni wheels with proportional speed scaling.
 * If any wheel exceeds PWM_MAX, all three are scaled down by the same
 * factor to preserve the intended velocity direction.
 */
void driveOmni(float w1, float w2, float w3) {
    int s1 = toPWM(w1);
    int s2 = toPWM(w2);
    int s3 = toPWM(w3);

    // Proportional scaling — keep direction ratios intact
    int maxSpeed = max({s1, s2, s3});
    if (maxSpeed > PWM_MAX) {
        float scale = (float)PWM_MAX / maxSpeed;
        s1 = (int)(s1 * scale);
        s2 = (int)(s2 * scale);
        s3 = (int)(s3 * scale);
    }

    setMotor(M1_DIR, M1_PWM, w1 >= 0, s1);
    setMotor(M2_DIR, M2_PWM, w2 >= 0, s2);
    setMotor(M3_DIR, M3_PWM, w3 >= 0, s3);
}


// ---------------------------------------------------------------------------
// Heading lock
// ---------------------------------------------------------------------------

/**
 * Compute corrective angular velocity from IMU heading error.
 * Uses proportional control only (Kp). A derivative term is omitted
 * because the IMU data arriving via EasyTransfer already has some
 * inherent filtering from the sensor fusion algorithm.
 */
double computeHeadingOmega(double currentAngle) {
    headingError = headingTarget - currentAngle;
    // Wrap to [-180, 180] to always take the shortest rotation path
    while (headingError >  180.0) headingError -= 360.0;
    while (headingError < -180.0) headingError += 360.0;
    return Kp_HEADING * headingError;
}


// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);

    ET_ps.begin(details(ps4), &Serial);
    ET_enc.begin(details(enc), &Serial3);

    analogWriteResolution(8);   // 8-bit PWM on Due

    // Motor and actuator pin modes
    int outputPins[] = {
        M1_PWM, M1_DIR, M2_PWM, M2_DIR, M3_PWM, M3_DIR,
        NET_PWM, NET_DIR, FAN_PWM, FAN_DIR,
        PNE_EXTEND, PNE_RETRACT
    };
    for (int pin : outputPins) {
        pinMode(pin, OUTPUT);
    }

    stopAllMotors();
    Serial.println("R2 defense controller ready");
}


// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    ET_ps.receiveData();
    ET_enc.receiveData();

    int  jx           = ps4.x;
    int  jy           = ps4.y;
    double currentAngle = enc.angle;

    // Compute corrective angular velocity from IMU
    omega = computeHeadingOmega(currentAngle);

    // -----------------------------------------------------------------------
    // Priority 1: Emergency stop (D-pad down)
    // -----------------------------------------------------------------------
    if (ps4.down) {
        stopAllMotors();
        return;
    }

    // -----------------------------------------------------------------------
    // Priority 2: Omnidirectional drive
    // Joystick deadzone prevents motor creep when stick is released.
    // -----------------------------------------------------------------------
    if (abs(jx) >= JOYSTICK_DEADZONE || abs(jy) >= JOYSTICK_DEADZONE) {

        // 3-wheel omni inverse kinematics
        // omega is the PID-corrected angular component (heading lock)
        float w1 = ( 2.0f * jx / 3.0f)                         + (omega / 3.0f);
        float w2 = (-      jx / 3.0f) + (jy / sqrt(3.0f))      + (omega / 3.0f);
        float w3 = (-      jx / 3.0f) - (jy / sqrt(3.0f))      + (omega / 3.0f);

        driveOmni(w1, w2, w3);
    }

    // -----------------------------------------------------------------------
    // Priority 3: In-place rotation (circle button)
    // Locks heading target to current angle + small increment
    // -----------------------------------------------------------------------
    else if (ps4.circle) {
        headingTarget += 30.0;   // degrees per press — adjust as needed
        float w_rot = omega / 3.0f;
        driveOmni(w_rot, w_rot, w_rot);
    }

    // -----------------------------------------------------------------------
    // Priority 4: Fan / defensive blower (R1)
    // -----------------------------------------------------------------------
    else if (ps4.r1) {
        setMotor(FAN_DIR, FAN_PWM, false, PWM_MAX);
    }

    // -----------------------------------------------------------------------
    // Priority 5: Pneumatics (square button)
    // -----------------------------------------------------------------------
    else if (ps4.square) {
        digitalWrite(PNE_EXTEND,  HIGH);
        delay(50);
        digitalWrite(PNE_RETRACT, HIGH);
    }

    // -----------------------------------------------------------------------
    // Priority 6: Sliding net mechanism (triangle — edge-triggered)
    // Run the net motor until the encoder reaches NET_TARGET_TICKS.
    // -----------------------------------------------------------------------
    else if (ps4.triangle && !trianglePrevState && !netRunning) {
        netRunning    = true;
        netStartTicks = netEncoder.read();
    }
    else if (netRunning) {
        long elapsed = abs(netEncoder.read() - netStartTicks);
        if (elapsed < NET_TARGET_TICKS) {
            setMotor(NET_DIR, NET_PWM, HIGH, NET_SPEED);
        } else {
            analogWrite(NET_PWM, 0);   // stop — target reached
            netRunning = false;
        }
    }

    // -----------------------------------------------------------------------
    // Default: idle
    // -----------------------------------------------------------------------
    else {
        stopAllMotors();
    }

    trianglePrevState = ps4.triangle;
}
