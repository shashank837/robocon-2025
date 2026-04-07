/**
 * upper_control.ino — R1 Offense: Upper Mechanism Controller
 * ===========================================================
 * Platform: ESP32 DevKit
 *
 * This firmware coordinates three independent actuators on R1's upper
 * mechanism from a single ESP32. All control is event-driven via serial
 * commands, allowing the ROS layer or an operator to command each
 * subsystem independently without blocking the others.
 *
 * Actuators
 * ---------
 *
 * 1. BLDC shooter motor (via ESC, PWM 50 Hz)
 *    ─────────────────────────────────────────
 *    The ESC requires a calibration sequence on every boot to learn
 *    the throttle range. After calibration, throttle is set by writing
 *    microsecond pulse widths in [1000, 2000]:
 *        1000 µs → minimum throttle (motor stopped / armed)
 *        2000 µs → maximum throttle
 *
 *    Serial command:  "T <microseconds>"
 *    Example:         "T 1500"  → 50% throttle
 *
 * 2. Stepper motor — shooter angle (closed-loop with encoder)
 *    ─────────────────────────────────────────────────────────
 *    A stepper motor adjusts the shooter's launch angle. An encoder
 *    on the driven shaft provides position feedback so the controller
 *    can verify movement and correct for missed steps.
 *
 *    Gear ratio: 8800 encoder ticks = 23 mechanical degrees
 *    Step rate: 150 µs high + 150 µs low per step (≈ 3300 steps/sec)
 *    Deadband: ±0.2° — steps are only issued when error exceeds this.
 *
 *    Serial command:  "A <degrees>"
 *    Example:         "A 30.5"  → move to 30.5 degrees
 *
 * 3. Pneumatic dribbler (4-channel solenoid valve)
 *    ───────────────────────────────────────────────
 *    A double-acting pneumatic cylinder drives the dribbler mechanism.
 *    Four solenoid valves (two H-bridge pairs) control extend / retract.
 *    The sequence is timed to allow the cylinder to complete each stroke
 *    before the valve state changes.
 *
 *    Serial command:  "D"  → trigger dribble sequence
 *
 * Serial commands summary
 * -----------------------
 *    "A <float>"   Set stepper target angle (degrees)
 *    "T <int>"     Set ESC throttle (1000–2000 µs)
 *    "D"           Trigger pneumatic dribble sequence
 */

#include <ESP32Encoder.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

// Stepper motor
#define PIN_STEP      19
#define PIN_DIR       21
#define PIN_ENC_A     22
#define PIN_ENC_B     23

// BLDC ESC
#define PIN_ESC       18

// Pneumatic dribbler (two H-bridge pairs)
#define PIN_PNE_A1    14   // Cylinder A — extend
#define PIN_PNE_A2    27   // Cylinder A — retract
#define PIN_PNE_B1    26   // Cylinder B — extend
#define PIN_PNE_B2    25   // Cylinder B — retract

// ---------------------------------------------------------------------------
// Stepper constants
// ---------------------------------------------------------------------------
const float STEPS_PER_DEGREE = 771.70;       // physical calibration constant
const float ENCODER_TICKS_PER_DEG = 8800.0 / 23.0;  // ticks per output degree
const float ANGLE_DEADBAND = 0.2f;           // degrees — ignore errors smaller than this
const int   STEP_PULSE_US  = 150;            // µs — step pulse half-period

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
ESP32Encoder stepperEncoder;
Servo        ESC;

float  currentAngle   = 0.0f;
float  targetAngle    = 0.0f;
bool   escCalibrated  = false;
bool   dribblePending = false;

String serialBuffer   = "";
bool   lineComplete   = false;


// ---------------------------------------------------------------------------
// Helper: move stepper to reduce angle error
// ---------------------------------------------------------------------------
void moveStepperToTarget() {
    long  encoderTicks = stepperEncoder.getCount();
    currentAngle       = (float)encoderTicks / ENCODER_TICKS_PER_DEG;
    float error        = targetAngle - currentAngle;

    if (abs(error) <= ANGLE_DEADBAND) return;

    long  stepsNeeded = (long)abs(error * STEPS_PER_DEGREE);
    bool  dirForward  = (error > 0);
    digitalWrite(PIN_DIR, dirForward ? LOW : HIGH);

    for (long i = 0; i < stepsNeeded; i++) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(STEP_PULSE_US);
    }

    // Verify final position via encoder
    long  finalTicks = stepperEncoder.getCount();
    float finalAngle = (float)finalTicks / ENCODER_TICKS_PER_DEG;
    Serial.printf("Stepper done — target: %.2f°  actual: %.2f°\n",
                  targetAngle, finalAngle);
}


// ---------------------------------------------------------------------------
// Helper: run pneumatic dribble sequence
// ---------------------------------------------------------------------------
void runDribbleSequence() {
    Serial.println("Dribble: starting sequence");

    // Extend cylinder A, hold cylinder B retracted
    digitalWrite(PIN_PNE_A2, LOW);
    digitalWrite(PIN_PNE_A1, HIGH);
    delay(2000);

    // Release B retract, extend B
    digitalWrite(PIN_PNE_B2, LOW);
    delay(100);
    digitalWrite(PIN_PNE_B1, HIGH);
    delay(750);

    // Retract B
    digitalWrite(PIN_PNE_B1, LOW);
    digitalWrite(PIN_PNE_B2, HIGH);
    delay(1500);

    // Retract A
    digitalWrite(PIN_PNE_A1, LOW);
    digitalWrite(PIN_PNE_A2, HIGH);
    delay(1500);

    // Home B
    digitalWrite(PIN_PNE_B2, LOW);
    digitalWrite(PIN_PNE_B1, HIGH);
    delay(1000);

    // Idle both
    digitalWrite(PIN_PNE_B1, LOW);
    digitalWrite(PIN_PNE_B2, HIGH);

    Serial.println("Dribble: sequence complete");
}


// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Stepper
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR,  OUTPUT);
    stepperEncoder.attachFullQuad(PIN_ENC_A, PIN_ENC_B);
    stepperEncoder.setCount(0);

    // Pneumatics — default: cylinder B retracted (safe idle)
    pinMode(PIN_PNE_A1, OUTPUT); pinMode(PIN_PNE_A2, OUTPUT);
    pinMode(PIN_PNE_B1, OUTPUT); pinMode(PIN_PNE_B2, OUTPUT);
    digitalWrite(PIN_PNE_B1, LOW);
    digitalWrite(PIN_PNE_B2, HIGH);

    // ESC calibration sequence
    // The ESC must see max throttle, then min throttle to learn the range.
    ESC.setPeriodHertz(50);
    ESC.attach(PIN_ESC, 1000, 2000);

    Serial.println("ESC: calibration — plug in battery now");
    ESC.writeMicroseconds(1100);   // max throttle signal
    delay(5000);
    ESC.writeMicroseconds(1000);   // min throttle — arms the ESC
    delay(5000);
    escCalibrated = true;
    Serial.println("ESC: calibrated and armed");
    Serial.println("Commands: 'A <deg>'  'T <us>'  'D'");
}


// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    // --- Serial command parser ---
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            lineComplete = true;
        } else {
            serialBuffer += c;
        }
    }

    if (lineComplete) {
        serialBuffer.trim();
        char cmd   = serialBuffer.charAt(0);
        String arg = serialBuffer.substring(1);
        arg.trim();

        if (cmd == 'A' || cmd == 'a') {
            targetAngle = arg.toFloat();
            Serial.printf("Stepper: target = %.2f°\n", targetAngle);

        } else if ((cmd == 'T' || cmd == 't') && escCalibrated) {
            int throttle = arg.toInt();
            if (throttle >= 1000 && throttle <= 2000) {
                ESC.writeMicroseconds(throttle);
                Serial.printf("ESC: throttle = %d µs\n", throttle);
            } else {
                Serial.println("ESC: invalid throttle — use 1000–2000");
            }

        } else if (cmd == 'D' || cmd == 'd') {
            dribblePending = true;

        } else {
            Serial.println("Unknown command — use 'A <deg>', 'T <us>', or 'D'");
        }

        serialBuffer = "";
        lineComplete  = false;
    }

    // --- Stepper position control ---
    moveStepperToTarget();

    // --- Pneumatic dribble sequence ---
    if (dribblePending) {
        dribblePending = false;
        runDribbleSequence();
    }

    delay(20);
}
