#!/usr/bin/env python3
"""
joystick_bridge.py — PS4 DualShock → ROS Twist
===============================================
ROS Noetic | Raspberry Pi 4

Translates raw /joy messages (sensor_msgs/Joy) from the joy node into
a scaled Twist message on /ps4_data consumed by motion_controller.py.

Axis mapping (standard PS4 / joy node)
---------------------------------------
  axes[0]  Left stick X   → lateral velocity  (vx), negated for correct frame
  axes[1]  Left stick Y   → forward velocity  (vy)
  axes[3]  Right stick X  → desired heading   (angular.z)

Scaling
-------
  Linear axes are scaled by 200 (maps [-1, 1] joystick to [-200, 200] speed
  units consumed by the IK solver). The heading axis is scaled by 20 to give
  a finer heading target resolution on the right stick.

  The sign inversion on axes[0] corrects for the physical mounting orientation
  of the left stick relative to the robot's body frame.
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

_pub = None

VELOCITY_SCALE = 200.0  # speed units per unit joystick deflection
HEADING_SCALE  =  20.0  # degrees per unit joystick deflection


def _joy_callback(msg):
    twist = Twist()
    twist.linear.x  = -msg.axes[0] * VELOCITY_SCALE   # lateral  (negated for frame)
    twist.linear.y  =  msg.axes[1] * VELOCITY_SCALE    # forward
    twist.angular.z =  msg.axes[3] * HEADING_SCALE     # desired heading target

    rospy.logdebug(f"vx={twist.linear.x:.1f}  vy={twist.linear.y:.1f}  "
                   f"heading_target={twist.angular.z:.1f}")
    _pub.publish(twist)


def main():
    global _pub
    rospy.init_node('joystick_bridge', anonymous=True)
    _pub = rospy.Publisher('/ps4_data', Twist, queue_size=10)
    rospy.Subscriber('/joy', Joy, _joy_callback)
    rospy.loginfo("joystick_bridge: listening on /joy")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
