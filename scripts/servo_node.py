#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from xiaoche.msg import SteeringAngle

from math import pi

from servoserial import ServoSerial


class SteeringGear(ServoSerial):
    YAW_MEDIAN = 1945
    YAW_MAX = 3310  # pi/2 left
    YAW_MIN = 580   # pi/2 right
    YAW_SCALE = (YAW_MAX - YAW_MIN) / pi

    PITCH_MEDIAN = 2900
    PITCH_MAX = 3800  # pi/3 up
    PITCH_MIN = 2400
    PITCH_SCALE = (PITCH_MAX - PITCH_MEDIAN) / (pi / 3)

    def __init__(self, yaw_joint, pitch_joint):
        super(SteeringGear, self).__init__()
        self.joints_pub = rospy.Publisher('~joint_states', JointState, queue_size=10)
        self.joints_msg = JointState()
        self.joints_msg.name = [yaw_joint, pitch_joint]

    def cleanup(self):
        super(SteeringGear, self).__del__()

    def set_angle(self, angle):
        yaw = angle.yaw * self.YAW_SCALE + self.YAW_MEDIAN
        yaw = int(min(self.YAW_MAX, max(self.YAW_MIN, yaw)))
        pitch = angle.pitch * self.PITCH_SCALE + self.PITCH_MEDIAN
        pitch = int(min(self.PITCH_MAX, max(self.PITCH_MIN, pitch)))
        self.Servo_serial_double_control(1, yaw, 2, pitch)
        # notify joint state update
        # TODO: do we need to clamp yaw/pitch?
        self.joints_msg.position = [
                (yaw - self.YAW_MEDIAN) / self.YAW_SCALE ,
                (pitch - self.PITCH_MEDIAN) / self.PITCH_SCALE ]
        self.joints_pub.publish(self.joints_msg)

    def center_service(self, req):
        #self.Servo_serial_double_control(1, self.YAW_MEDIAN, 2, self.PITCH_MEDIAN)
        self.set_angle(SteeringAngle(0.0, 0.0))
        return EmptyResponse()


def main():
    rospy.init_node('servo_node')

    # get two revolute joints name
    yaw_joint = rospy.get_param('~yaw_joint', "gimbal_hservo_vservo_joint")
    pitch_joint = rospy.get_param('~pitch_joint', "gimbal_vservo_handle_joint")

    device = SteeringGear(yaw_joint, pitch_joint)

    # indicate limits
    # you should get these params after /servo_node/center service is available
    rospy.set_param('~yaw_max', (device.YAW_MAX - device.YAW_MEDIAN) / device.YAW_SCALE)
    rospy.set_param('~yaw_min', (device.YAW_MIN - device.YAW_MEDIAN) / device.YAW_SCALE)
    rospy.set_param('~pitch_max', (device.PITCH_MAX - device.PITCH_MEDIAN) / device.PITCH_SCALE)
    rospy.set_param('~pitch_min', (device.PITCH_MIN - device.PITCH_MEDIAN) / device.PITCH_SCALE)

    rospy.Subscriber('servo_angle', SteeringAngle, device.set_angle)
    rospy.Service('~center', Empty, device.center_service)

    rospy.spin()
    device.cleanup()


if __name__ == '__main__':
    main()
