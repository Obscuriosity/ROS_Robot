#! /usr/bin/env python
# subscribe to encoder values (state)
# publish state
# subscribe to control_effort and pass to motors

import rospy
from DCMotor import DCMotor as DCM
from std_msgs.msg import UInt64

leftM = DCM(22, 23)


def encCallback(enclf):
    stateData = enclf

def controlMotor(args):
    rospy.init_node('motorControl')
    rate = rospy.Rate(10)  # 10hz
    rospy.Subscriber('/enc_lf', UInt64, encCallback)
    pub = rospy.Publisher('state', UInt64, queue_size=10)
    rospy.spin()
    while not rospy.is_shutdown():
        STATEdata = stateData
        pub.publish(STATEdata)
        rate.sleep()


if __name__ == '__main__':
    try:
        controlMotor()
    except rospy.ROSInterruptException:
        pass
