#!/usr/bin/env python3
"""
encoder_bridge.py — USART Encoder Bridge (ESP32 → ROS)
=======================================================
ROS Noetic | Raspberry Pi 4

Reads structured encoder packets from the ESP32 odometry firmware over
USART (/dev/serial0, 115200 baud) and republishes them as a ROS
Float32MultiArray on /encoder_distance.

Packet format (ASCII, newline-terminated)
-----------------------------------------
  "X:<int>, Y:<int>"
  e.g.  "X:4820, Y:-1203"

  Values are raw quadrature encoder counts. They are converted to
  centimetres using the wheel calibration constant:
      1 revolution = 10,000 pulses = 18.84 cm  (wheel circumference)

  The X axis is sign-inverted to match the robot's body-frame convention
  (positive X = robot forward in the field frame).

Published topic
---------------
  /encoder_distance  (Float32MultiArray)
      data[0] : X distance in centimetres
      data[1] : Y distance in centimetres
"""

import rospy
import serial
from std_msgs.msg import Float32MultiArray

# 10,000 pulses per revolution; wheel circumference = 18.84 cm
PULSES_PER_REV    = 10_000
WHEEL_CIRC_CM     = 18.84
CM_PER_PULSE      = WHEEL_CIRC_CM / PULSES_PER_REV   # 0.001884 cm/pulse

SERIAL_PORT  = '/dev/serial0'
SERIAL_BAUD  = 115_200
PUBLISH_RATE = 50  # Hz


def _parse_packet(raw_bytes):
    """
    Decode a USART packet and return (x_cm, y_cm) or None on parse failure.
    """
    try:
        line  = raw_bytes.decode('utf-8', errors='ignore').strip()
        parts = line.split(',')
        if len(parts) != 2:
            return None
        if ':' not in parts[0] or ':' not in parts[1]:
            return None

        x_raw = int(parts[0].split(':')[1].strip())
        y_raw = int(parts[1].split(':')[1].strip())

        x_cm  = -x_raw * CM_PER_PULSE   # sign inversion for body-frame convention
        y_cm  =  y_raw * CM_PER_PULSE

        return x_cm, y_cm

    except (ValueError, IndexError):
        return None


def main():
    rospy.init_node('encoder_bridge')
    pub  = rospy.Publisher('/encoder_distance', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)

    ser = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=0.1)
    rospy.loginfo(f"encoder_bridge: listening on {SERIAL_PORT} @ {SERIAL_BAUD}")

    while not rospy.is_shutdown():
        if ser.in_waiting:
            raw    = ser.readline()
            result = _parse_packet(raw)
            if result is not None:
                x_cm, y_cm = result
                msg      = Float32MultiArray()
                msg.data = [x_cm, y_cm]
                pub.publish(msg)
                rospy.logdebug(f"encoder: X={x_cm:.2f} cm  Y={y_cm:.2f} cm")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
