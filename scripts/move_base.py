#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

rospy.set_param('/joy_node/autorepeat_rate', 20.0)

angle_axis = 2
speed_axis = 1
speed_min  = 0
speed_max  = 20000
angle_pub  = rospy.Publisher('/commands/servo/position', Float64, queue_size = 1)
speed_pub  = rospy.Publisher('/commands/motor/speed', Float64, queue_size = 1)

def joy_callback(data):
    angle_req = Float64()
    speed_req = Float64()
    angle_req.data = 1.0 - (data.axes[angle_axis] + 1.0)/2.0
    speed_req.data = data.axes[speed_axis] * speed_max
    angle_pub.publish(angle_req)
    speed_pub.publish(speed_req)

if __name__ == '__main__':
    try:
        rospy.init_node('move_base', anonymous = True)
        rospy.Subscriber('/joy', Joy, joy_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
