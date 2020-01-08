#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

global current_command_topic

car_name              = str(sys.argv[1])
joy_angle_axis        = 2
joy_angle_scaler      = 100.0
joy_speed_axis        = 1
joy_speed_scaler      = 100.0
ctl_offboard_button   = 7
ctl_teleop_button     = 6
command_topic         = ['offboard/command',
                         'teleop/command']
log_message           = 'control priority assigned to - {}'
control_priority      = ['JTX2_OFFBOARD',
                         'REMOTE_LOGITECH']
current_command_topic = 'teleop/command'
message_display       = [False, False]
multiplexer_pub       = rospy.Publisher('/{}/multiplexer/command'.format(car_name), AckermannDrive, queue_size = 1)

def offboard_passthrough(data):
    multiplexer_pub.publish(data)

def joy_command_callback(data):
    global current_command_topic
    passthrough_command = AckermannDrive()
    # listen to control transfer commands
    if data.buttons[ctl_offboard_button] and not data.buttons[ctl_teleop_button]:
        if not message_display[0]:
            rospy.loginfo(log_message.format(control_priority[0]))
            message_display[0] = True
            message_display[1] = False
        if current_command_topic != command_topic[0]:
            current_command_topic = command_topic[0]
    if not data.buttons[ctl_offboard_button] and data.buttons[ctl_teleop_button]:
        if not message_display[1]:
            rospy.loginfo(log_message.format(control_priority[1]))
            message_display[0] = False
            message_display[1] = True
        if current_command_topic != command_topic[1]:
            current_command_topic = command_topic[1]
    if current_command_topic == command_topic[1]:
        # identify and scale raw command data
        passthrough_command.steering_angle = -1.0 * data.axes[joy_angle_axis] * joy_angle_scaler
        passthrough_command.speed          = data.axes[joy_speed_axis] * joy_speed_scaler
        multiplexer_pub.publish(passthrough_command)
    else:
        rospy.Subscriber('/{}/{}'.format(car_name, command_topic[0]), AckermannDrive, offboard_passthrough)


if __name__ == '__main__':
    try:
        rospy.init_node('command_multiplexer', anonymous = True)
        rospy.Subscriber('/joy', Joy, joy_command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
