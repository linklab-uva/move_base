#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive

global curr_speed
global start_flag

car_name          = str(sys.argv[1])    # namespace handler (if using multi-vehicle setup)
priority_control  = str(sys.argv[2])    # default control priority assignment during start-up

angle_pub         = rospy.Publisher('/{}/commands/servo/position'.format(car_name), Float64, queue_size = 1)
speed_pub         = rospy.Publisher('/{}/commands/motor/speed'.format(car_name),    Float64, queue_size = 1)

curr_speed        = 0.0                 # store current speed for P-type control
start_flag        = False               # display diagnostics message on console (if enabled)

angle_min_rel     = -100.0              # max left command
angle_max_rel     = 100.0               # max right command
angle_min_abs     = 0.0                 # max left VESC servo
angle_max_abs     = 1.0                 # max right VESC servo

speed_min_rel     = -100.0              # min speed command
speed_max_rel     = 100.0               # max speed command
speed_min_abs     = -20000.0            # min speed VESC motor
speed_max_abs     = 20000.0             # max speed VESC motor
speed_change_step = 2000.0              # acceleration P-type control

def output_angle_mixer(rel_angle):
    output_angle = (rel_angle - angle_min_rel)/(angle_max_rel - angle_min_rel)
    output_angle = output_angle * (angle_max_abs - angle_min_abs)
    return output_angle

def output_speed_mixer(rel_speed):
    global curr_speed
    output_speed = (rel_speed - speed_min_rel)/(speed_max_rel - speed_min_rel)
    output_speed = output_speed * (speed_max_abs - speed_min_abs) - speed_max_abs
    if output_speed >= curr_speed + speed_change_step:
        curr_speed = curr_speed + speed_change_step
    elif output_speed <= curr_speed - speed_change_step:
        curr_speed = curr_speed - speed_change_step
    if abs(curr_speed) < speed_change_step:
        curr_speed = 0.0
    return curr_speed

def command_callback(data):
    angle_req = Float64()
    speed_req = Float64()
    angle_req.data = output_angle_mixer(data.steering_angle)
    speed_req.data = output_speed_mixer(data.speed)
    angle_pub.publish(angle_req)
    speed_pub.publish(speed_req)

if __name__ == '__main__':
    try:
        global start_flag
        rospy.init_node('move_base', anonymous = True)
        if not start_flag:
            start_flag = False
            rospy.set_param('/{}/command_state'.format(car_name), priority_control)
            rospy.loginfo('starting move_base node with default: {}'.format(priority_control))
        if rospy.get_param('/{}/command_state'.format(car_name)) == 'priority_teleop':
            rospy.loginfo('move_base listening to: priority_teleop')
            rospy.Subscriber('/{}/teleop/command'.format(car_name), AckermannDrive, command_callback)
        elif rospy.get_param('/{}/command_state'.format(car_name)) == 'priority_offboard':
            rospy.loginfo('move_base listening to: priority_offboard')
            rospy.Subscriber('/{}/offboard/command'.format(car_name), AckermannDrive, command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
