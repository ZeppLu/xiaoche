#!/usr/bin/env python

import rospy
from yahboom.msg import SteeringAngle

from servoserial import ServoSerial


class SteeringGear(ServoSerial):
    YAW_MEDIAN = 1925
    YAW_MAX = 2850
    YAW_MIN = 1000
    PITCH_MEDIAN = 2080
    PITCH_MAX = 2620
    PITCH_MIN = 1540

    ANGLE_SCALE = (2850-1925) / (3.14159265/3)

    def __init__(self):
        super(SteeringGear, self).__init__()

    def cleanup(self):
        super(SteeringGear, self).__del__()

    def set_angle(self, angle):
        yaw = angle.yaw * self.ANGLE_SCALE + self.YAW_MEDIAN
        yaw = int(min(self.YAW_MAX, max(self.YAW_MIN, yaw)))
        pitch = angle.pitch * self.ANGLE_SCALE + self.PITCH_MEDIAN
        pitch = int(min(self.PITCH_MAX, max(self.PITCH_MIN, pitch)))
        self.Servo_serial_double_control(1, yaw, 2, pitch)


def main():
    rospy.init_node('servo_node')

    device = SteeringGear()
    rospy.Subscriber('servo_angle', SteeringAngle, device.set_angle)

    rospy.spin()
    device.cleanup()


if __name__ == '__main__':
    main()
