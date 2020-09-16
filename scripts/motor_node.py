#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from motor import Motor


def main():
    rospy.init_node('motor_node') 

    motor = Motor()
    rospy.Subscriber('platform_up_vel', Float32, motor.up)

    rospy.spin()
    motor.cleanup()


if __name__ == '__main__':
    main()
