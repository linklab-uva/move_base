#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

car_name            = str(sys.argv[1])
joy_angle_axis      = 2
joy_angle_scaler    = 100.0
joy_speed_axis      = 1
joy_speed_scaler    = 100.0
ctl_offboard_button = 7
ctl_teleop_button   = 6
command_state       = ['priority_teleop',
                       'priority_offboard']
joy_command_pub     = rospy.Publisher('/{}/teleop/command'.format(car_name), AckermannDrive, queue_size = 1)

def joy_command_callback(data):
    # identify and scale raw command data
    joy_command                = AckermannDrive()
    joy_command.steering_angle = -1.0 * data.axes[joy_angle_axis] * joy_angle_scaler
    joy_command.speed          = data.axes[joy_speed_axis] * joy_speed_scaler
    joy_command_pub.publish(joy_command)
    # listen to control transfer commands
    if data.buttons[ctl_offboard_button] and not data.buttons[ctl_teleop_button]:
        rospy.set_param('/{}/command_state'.format(car_name), command_state[1])
    if not data.buttons[ctl_offboard_button] and data.buttons[ctl_teleop_button]:
        rospy.set_param('/{}/command_state'.format(car_name), command_state[0])

if __name__ == '__main__':
    try:
        rospy.init_node('remote_control', anonymous = True)
        rospy.Subscriber('/joy', Joy, joy_command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
