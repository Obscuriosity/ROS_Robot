#! /usr/bin/env python
# subscribe to encoder values (state)
# publish state
# subscribe to control_effort and pass to motors

import rospy
from DCMotor import DCMotor as DCM
from std_msgs.msg import UInt64, Float64

rospy.init_node('motorControl')
prevStatedata = 0
leftM = DCM(22, 23)


def encoderCB(enclf):
    global prevStatedata
    stateData = enclf.data - prevStatedata
    prevStatedata = stateData
    pub.publish(stateData)


def motorCB(pwm):
    leftM.run(pwm)


rate = rospy.Rate(10)  # 10hz
encoderSub = rospy.Subscriber('enc_lf', UInt64, encoderCB)
motorSub = rospy.Subscriber('control_effort', Float64, motorCB)
pub = rospy.Publisher('state', UInt64, queue_size=10)

rospy.spin()
rate.sleep()
