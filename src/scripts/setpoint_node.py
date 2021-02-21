#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64


def setpoint():
    pub = rospy.Publisher('setpoint', Float64, queue_size=10)
    rospy.init_node('setpoint_node')
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        SETPOINT = 20
        rospy.loginfo(SETPOINT)
        pub.publish(SETPOINT)
        rate.sleep()


if __name__ == '__main__':
    try:
        setpoint()
    except rospy.ROSInterruptException:
        pass
