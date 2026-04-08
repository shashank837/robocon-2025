#!/usr/bin/env python3
"""
motor_driver.py — R1 Hardware Abstraction Layer
================================================
ROS Noetic | Raspberry Pi 4 | pigpio

Bridges the ROS control layer to physical hardware on R1:
  - 3× brushed DC motors (drive) via pigpio PWM + direction pins
  - Pneumatic dribbler valves via GPIO
  - BLDC shooter relay via GPIO

Topic interfaces
----------------
  Subscribed:
    /motor_value    (Int32MultiArray) — signed wheel speeds [-255, 255]
    /joy            (Joy)             — raw PS4 for actuator buttons
    /bno055_data    (Float32)         — IMU yaw (logged only)

  All hardware is zeroed safely on node shutdown.

Motor convention
----------------
  Positive value → direction pin HIGH, PWM = |value|
  Negative value → direction pin LOW,  PWM = |value|
  PWM range clamped to [0, 255] before writing to pigpio.

Pneumatics (dribbler)
---------------------
  L2/R2 trigger axis (axes[-1]):
    +1  → valve A open (extend)
    -1  → valve B open (retract)
     0  → both valves closed
  Two-valve design allows both forward and reverse pneumatic actuation.

Shooter (BLDC)
--------------
  Triangle → relay forward  (shooter ON)
  Cross    → relay reverse  (shooter OFF / reverse)
  Neither  → relay floating (shooter idle)
"""

import rospy
import pigpio
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Joy


# ---------------------------------------------------------------------------
# GPIO pin assignments (BCM numbering)
# ---------------------------------------------------------------------------

# Drive motors: (PWM pin, direction pin) — 4-wheel omni layout
MOTOR_PINS = [
    {"pwm": 21, "dir": 20},   # Motor 1 — front-left
    {"pwm": 16, "dir": 12},   # Motor 2 — front-right
    {"pwm": 13, "dir":  6},   # Motor 3 — rear-left
    {"pwm": 19, "dir": 26},   # Motor 4 — rear-right
]

# Pneumatic dribbler
PNEUMATIC_VALVE_A    = 19   # extend
PNEUMATIC_DRIBBLER_PWM = 26 # enables pneumatic supply pump

# BLDC shooter relay
SHOOTER_FWD = 23
SHOOTER_REV = 24

# ---------------------------------------------------------------------------
# pigpio initialisation
# ---------------------------------------------------------------------------
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("motor_driver: pigpio daemon not running — start with 'sudo pigpiod'")
    raise SystemExit(1)

# Configure all motor pins
for _m in MOTOR_PINS:
    pi.set_mode(_m["pwm"], pigpio.OUTPUT)
    pi.set_mode(_m["dir"], pigpio.OUTPUT)

# Configure actuator pins
for _pin in [PNEUMATIC_VALVE_A, PNEUMATIC_DRIBBLER_PWM, SHOOTER_FWD, SHOOTER_REV]:
    pi.set_mode(_pin, pigpio.OUTPUT)


# ---------------------------------------------------------------------------
# Motor control helpers
# ---------------------------------------------------------------------------

def _set_motor(motor: dict, signed_speed: int):
    """
    Drive a single motor.

    Parameters
    ----------
    motor        : dict with keys 'pwm' and 'dir' (GPIO pin numbers)
    signed_speed : integer in [-255, 255]. Sign = direction, magnitude = speed.
    """
    direction = pigpio.HIGH if signed_speed >= 0 else pigpio.LOW
    duty      = max(0, min(255, abs(signed_speed)))
    pi.write(motor["dir"], direction)
    pi.set_PWM_dutycycle(motor["pwm"], duty)


def _stop_all_motors():
    for motor in MOTOR_PINS:
        pi.set_PWM_dutycycle(motor["pwm"], 0)
        pi.write(motor["dir"], pigpio.LOW)


# ---------------------------------------------------------------------------
# ROS callbacks
# ---------------------------------------------------------------------------

def _motor_value_callback(msg):
    """
    Receive signed wheel speeds from motion_controller.py and
    drive each motor accordingly.
    """
    if len(msg.data) != 4:
        rospy.logwarn(f"motor_driver: expected 4 wheel speeds, got {len(msg.data)}")
        return

    for motor, speed in zip(MOTOR_PINS, msg.data):
        _set_motor(motor, int(speed))


def _joy_callback(msg):
    """
    Map PS4 button / axis state to pneumatic and BLDC shooter outputs.

    Axis layout (standard joy node mapping):
        axes[-1]   → L2/R2 combined trigger axis
        buttons[3] → Triangle
        buttons[0] → Cross
    """
    # --- Pneumatic dribbler ---
    trigger = msg.axes[-1] if msg.axes else 0.0

    if trigger == -1.0:       # R2 pressed → extend
        pi.write(PNEUMATIC_VALVE_A, pigpio.HIGH)
        pi.set_PWM_dutycycle(PNEUMATIC_DRIBBLER_PWM, 255)
    elif trigger == 1.0:      # L2 pressed → retract
        pi.write(PNEUMATIC_VALVE_A, pigpio.LOW)
        pi.set_PWM_dutycycle(PNEUMATIC_DRIBBLER_PWM, 255)
    else:                     # released → close supply
        pi.write(PNEUMATIC_VALVE_A, pigpio.LOW)
        pi.set_PWM_dutycycle(PNEUMATIC_DRIBBLER_PWM, 0)

    # --- BLDC shooter relay ---
    if len(msg.buttons) > 3:
        if msg.buttons[3]:    # Triangle → shoot
            pi.write(SHOOTER_FWD, pigpio.HIGH)
            pi.write(SHOOTER_REV, pigpio.LOW)
        elif msg.buttons[0]:  # Cross → reverse / stop
            pi.write(SHOOTER_FWD, pigpio.LOW)
            pi.write(SHOOTER_REV, pigpio.HIGH)
        else:                 # idle
            pi.write(SHOOTER_FWD, pigpio.LOW)
            pi.write(SHOOTER_REV, pigpio.LOW)


# ---------------------------------------------------------------------------
# Shutdown hook
# ---------------------------------------------------------------------------

def _shutdown():
    """Zero all outputs safely before the node exits."""
    rospy.loginfo("motor_driver: shutdown — zeroing all outputs")
    _stop_all_motors()
    pi.write(PNEUMATIC_VALVE_A, pigpio.LOW)
    pi.set_PWM_dutycycle(PNEUMATIC_DRIBBLER_PWM, 0)
    pi.write(SHOOTER_FWD, pigpio.LOW)
    pi.write(SHOOTER_REV, pigpio.LOW)
    pi.stop()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rospy.init_node('motor_driver')
    rospy.on_shutdown(_shutdown)

    rospy.Subscriber('/motor_value', Int32MultiArray, _motor_value_callback)
    rospy.Subscriber('/joy',         Joy,             _joy_callback)

    rospy.loginfo("motor_driver: running")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        _shutdown()
