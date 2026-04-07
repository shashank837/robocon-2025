#!/usr/bin/env python3
"""
imu_pipeline.py — BNO055 IMU Publisher with Dual-Stage Filter
=============================================================
ROS Noetic | Raspberry Pi 4 | Runs at 50 Hz

Reads orientation quaternions from the Bosch BNO055 9-DOF IMU over I2C,
converts to yaw angle, and applies a two-stage filter pipeline before
publishing the cleaned yaw on /bno055_data.

Filter pipeline
---------------
Stage 1 — Median filter (window = 5 samples):
    Rejects impulsive spikes caused by I2C glitches or momentary sensor
    noise. The median is insensitive to single outlier values in the
    window, making it ideal for spike rejection.

Stage 2 — Exponential low-pass filter (alpha = 0.1):
    Smooths residual high-frequency jitter. A low alpha (0.1) produces a
    heavily smoothed signal with ~10-sample lag — acceptable for a 50 Hz
    heading lock loop where yaw changes slowly relative to loop rate.

Fault handling
--------------
If the I2C bus returns None or the connection drops (OSError), the node
logs a warning and attempts automatic reconnection every 1 second. This
handles sensor brown-outs during high-current drive events without
requiring a node restart.
"""

import rospy
from std_msgs.msg import Float32

import board
import busio
import adafruit_bno055

import math
import time
from collections import deque


# ---------------------------------------------------------------------------
# Filter parameters
# ---------------------------------------------------------------------------
MEDIAN_WINDOW = 5    # samples — wider window = more spike rejection, more lag
LPF_ALPHA     = 0.1  # [0, 1] — lower = smoother but slower to respond
PUBLISH_RATE  = 50   # Hz


def _quaternion_to_yaw(w, x, y, z):
    """
    Extract yaw (rotation about world Z-axis) from a unit quaternion.

    Uses the standard ZYX Euler decomposition:
        yaw = atan2(2(wz + xy), 1 - 2(y² + z²))

    Returns yaw in degrees in the range [-180, 180].
    The BNO055 in NDOF fusion mode outputs calibrated quaternions that
    are already compensated for gyro bias and magnetic disturbances.
    """
    yaw_rad = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )
    return math.degrees(yaw_rad)


def _get_median(window):
    """Return the median of a deque of floats."""
    sorted_vals = sorted(window)
    n = len(sorted_vals)
    mid = n // 2
    return sorted_vals[mid] if n % 2 == 1 else (sorted_vals[mid - 1] + sorted_vals[mid]) / 2.0


def _try_connect():
    """
    Attempt to initialise the BNO055 over I2C.
    Returns the sensor object on success, None on failure.
    """
    try:
        i2c    = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
        rospy.loginfo("imu_pipeline: BNO055 connected")
        return sensor
    except Exception as exc:
        rospy.logwarn(f"imu_pipeline: BNO055 init failed — {exc}")
        return None


def main():
    rospy.init_node('imu_pipeline', anonymous=True)
    pub = rospy.Publisher('/bno055_data', Float32, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)

    sensor        = _try_connect()
    median_window = deque(maxlen=MEDIAN_WINDOW)
    filtered_yaw  = None  # initialised on first valid sample

    while not rospy.is_shutdown():

        # Reconnect if sensor was lost
        if sensor is None:
            sensor = _try_connect()
            time.sleep(1.0)
            rate.sleep()
            continue

        try:
            quat = sensor.quaternion
            if quat is None or any(v is None for v in quat):
                rospy.logwarn_throttle(5.0, "imu_pipeline: quaternion unavailable")
                rate.sleep()
                continue

            w, x, y, z = quat
            raw_yaw     = _quaternion_to_yaw(w, x, y, z)

            # --- Stage 1: median filter ---
            median_window.append(raw_yaw)
            median_yaw = _get_median(median_window)

            # --- Stage 2: exponential low-pass ---
            if filtered_yaw is None:
                filtered_yaw = median_yaw  # seed on first sample
            else:
                filtered_yaw = LPF_ALPHA * median_yaw + (1.0 - LPF_ALPHA) * filtered_yaw

            pub.publish(Float32(data=filtered_yaw))

        except OSError as exc:
            rospy.logwarn(f"imu_pipeline: I2C read error — {exc}. Reconnecting...")
            sensor = None
            time.sleep(1.0)

        except Exception as exc:
            rospy.logerr(f"imu_pipeline: unexpected error — {exc}")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
