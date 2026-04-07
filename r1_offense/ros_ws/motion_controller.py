#!/usr/bin/env python3
"""
motion_controller.py — R1 Offense Robot: Core Motion Controller
================================================================
ROS Noetic | Raspberry Pi 4 | Runs at 50 Hz

This node is the central control loop for R1. It fuses:
  - Driver velocity commands (from joystick_bridge.py via /ps4_data)
  - IMU heading (from imu_pipeline.py via /bno055_data)
  - Encoder odometry (from encoder_bridge.py via /encoder_distance)

And produces:
  - Wheel speed commands (to motor_driver.py via /motor_value)
  - Corrected odometry position (via /encoder)

Key behaviours
--------------
Field-centric drive:
    The driver's joystick always commands motion relative to the field.
    The IMU yaw is used to rotate the world-frame velocity vector into
    the robot's body frame before computing wheel speeds. The robot
    drives identically regardless of its physical heading.

Active Heading Lock (shared autonomy):
    The driver's right stick sets a desired heading target. A PID loop
    running at 50 Hz computes a corrective angular velocity (omega) that
    keeps the robot on that heading even during aggressive lateral moves.
    This eliminates rotational drift without requiring the driver to
    actively steer.

3-wheel omni inverse kinematics:
    Body-frame (vx, vy, omega) is decomposed analytically into three
    individual wheel speeds. Speeds are clipped to the PWM range by the
    downstream motor_driver node.
"""

import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray
from geometry_msgs.msg import Twist

# ---------------------------------------------------------------------------
# Robot geometry
# ---------------------------------------------------------------------------
WHEEL_RADIUS  = 0.075   # metres
ROBOT_RADIUS  = 0.2675  # metres — wheel contact circle radius

# ---------------------------------------------------------------------------
# PID tuning (heading lock)
# Kp dominates; Ki removes steady-state drift; Kd is left at 0 to avoid
# amplifying IMU noise.
# ---------------------------------------------------------------------------
Kp = 3.0
Ki = 0.05
Kd = 0.0

# Anti-windup: cap the integral term to prevent runaway on long heading errors
INTEGRAL_LIMIT = 50.0

# ---------------------------------------------------------------------------
# Module-level state (updated by ROS callbacks)
# ---------------------------------------------------------------------------
_pub         = None   # /motor_value publisher
_encoder_pub = None   # /encoder publisher

_current_yaw  = 0.0  # degrees — latest filtered IMU reading
_vx_user      = 0.0  # world-frame X velocity from driver (arbitrary units)
_vy_user      = 0.0  # world-frame Y velocity from driver (arbitrary units)
_target_yaw   = 0.0  # desired heading in degrees (driver-controlled)

# Encoder odometry accumulators
_prev_enc_x = 0.0
_prev_enc_y = 0.0
_dist_x     = 0.0
_dist_y     = 0.0

# PID state
_pid_integral   = 0.0
_pid_last_error = 0.0
_last_time      = None


# ---------------------------------------------------------------------------
# Kinematics helpers
# ---------------------------------------------------------------------------

def _rotate_to_robot_frame(vx_world, vy_world, yaw_deg):
    """
    Rotate a world-frame velocity vector into the robot's body frame.

    The robot's body-X axis points at angle `yaw_deg` in the world frame.
    This rotation allows the driver's joystick axes (world frame) to map
    correctly to motor commands regardless of the robot's current heading.
    """
    yaw_rad = math.radians(yaw_deg)
    vx_body =  vx_world * math.cos(yaw_rad) + vy_world * math.sin(yaw_rad)
    vy_body = -vx_world * math.sin(yaw_rad) + vy_world * math.cos(yaw_rad)
    return vx_body, vy_body


def _omni3_inverse_kinematics(vx, vy, omega):
    """
    3-wheel symmetric omni-drive inverse kinematics.

    Wheel layout (angles from robot front):
        Motor 1: 90°  (left)
        Motor 2: 210° (right-rear)
        Motor 3: 330° (left-rear)

    Returns raw (unscaled) speed for each wheel. The sign encodes direction.
    """
    w1 = (-vx / 3.0)        + (vy / math.sqrt(3)) + (omega / 3.0)
    w2 = (-vx / 3.0)        - (vy / math.sqrt(3)) + (omega / 3.0)
    w3 = ( 2.0 * vx / 3.0)                         + (omega / 3.0)
    return [w1, w2, w3]


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

def _pid_heading(error, dt):
    """
    Proportional-Integral-Derivative controller for heading lock.

    Returns the corrective angular velocity (omega) to apply.
    The integral term is clamped to INTEGRAL_LIMIT to prevent windup
    during large sustained heading errors (e.g. robot spinning freely).
    """
    global _pid_integral, _pid_last_error

    _pid_integral += error * dt
    _pid_integral  = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, _pid_integral))

    derivative = (error - _pid_last_error) / dt if dt > 1e-6 else 0.0
    output     = Kp * error + Ki * _pid_integral + Kd * derivative

    _pid_last_error = error
    return output


# ---------------------------------------------------------------------------
# ROS callbacks
# ---------------------------------------------------------------------------

def _imu_callback(msg):
    """Receive latest filtered yaw (degrees) from imu_pipeline.py."""
    global _current_yaw
    _current_yaw = msg.data


def _ps4_callback(msg):
    """
    Receive driver commands from joystick_bridge.py.

    msg.linear.x  → world-frame forward velocity
    msg.linear.y  → world-frame lateral velocity
    msg.angular.z → desired heading target (degrees, from right stick)
    """
    global _vx_user, _vy_user, _target_yaw
    _vx_user    = msg.linear.x
    _vy_user    = msg.linear.y
    _target_yaw = msg.angular.z


def _encoder_callback(msg):
    """
    Receive raw encoder distances (cm) from encoder_bridge.py.

    Projects the incremental encoder displacement onto the current
    commanded velocity direction to accumulate a position estimate
    in world-frame X and Y. This is a simple dead-reckoning approach
    that works well when the velocity direction is known.
    """
    global _prev_enc_x, _prev_enc_y, _dist_x, _dist_y

    new_x = msg.data[0]
    new_y = msg.data[1]

    delta_x = new_x - _prev_enc_x
    delta_y = new_y - _prev_enc_y
    _prev_enc_x = new_x
    _prev_enc_y = new_y

    # Rotate encoder deltas into the world frame using current IMU yaw
    yaw_rad     = math.radians(_current_yaw)
    dx_world    = delta_x * math.cos(yaw_rad) - delta_y * math.sin(yaw_rad)
    dy_world    = delta_x * math.sin(yaw_rad) + delta_y * math.cos(yaw_rad)

    # Only accumulate position along the commanded direction to suppress
    # cross-talk from wheel slip on perpendicular axes
    cmd_mag = math.hypot(_vx_user, _vy_user)
    if cmd_mag > 0.01:
        ux = _vx_user / cmd_mag
        uy = _vy_user / cmd_mag
        proj    = dx_world * ux + dy_world * uy
        _dist_x += proj * ux
        _dist_y += proj * uy

    out = Float32MultiArray()
    out.data = [_dist_x, _dist_y]
    _encoder_pub.publish(out)


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

def _control_loop():
    """
    50 Hz loop: heading PID → field-centric transform → omni IK → publish.
    """
    global _last_time

    rate      = rospy.Rate(50)
    _last_time = rospy.get_time()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        dt  = now - _last_time if _last_time is not None else 0.02
        _last_time = now

        # --- Heading lock PID ---
        yaw_error = _target_yaw - _current_yaw
        # Wrap error to [-180, 180] to take the shortest angular path
        yaw_error = (yaw_error + 180.0) % 360.0 - 180.0
        omega     = _pid_heading(yaw_error, dt)

        # --- Field-centric velocity transform ---
        vx_body, vy_body = _rotate_to_robot_frame(_vx_user, _vy_user, _current_yaw)

        # --- 3-wheel inverse kinematics ---
        wheel_speeds = _omni3_inverse_kinematics(vx_body, vy_body, omega)

        # Publish as integer PWM values (clipping happens in motor_driver.py)
        msg      = Int32MultiArray()
        msg.data = [int(round(s)) for s in wheel_speeds]
        _pub.publish(msg)

        rate.sleep()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global _pub, _encoder_pub

    rospy.init_node('motion_controller')

    _pub         = rospy.Publisher('/motor_value',      Int32MultiArray,  queue_size=10)
    _encoder_pub = rospy.Publisher('/encoder',          Float32MultiArray, queue_size=10)

    rospy.Subscriber('/ps4_data',         Twist,            _ps4_callback)
    rospy.Subscriber('/bno055_data',      Float32,          _imu_callback)
    rospy.Subscriber('/encoder_distance', Float32MultiArray, _encoder_callback)

    rospy.loginfo("motion_controller: started at 50 Hz")
    _control_loop()


if __name__ == '__main__':
    main()
