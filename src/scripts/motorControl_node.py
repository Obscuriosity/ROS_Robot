#! /usr/bin/env python
# subscribe to encoder values (state)
# publish state
# subscribe to control_effort and pass to motors

import rospy
from DCMotor import DCMotor as DCM
from std_msgs.msg import Float64

rospy.init_node('motorControl')
leftprevStateData = 0.0
rightprevStateData = 0.0
leftPWM = 0.0
leftM = DCM(22, 23)
rightPWM = 0.0
rightM = DCM(27, 18)


def lfencCB(enclf):
    global leftprevStateData
    stateData = enclf.data - leftprevStateData
    leftprevStateData = enclf.data
    print("L previous ", leftprevStateData, "State", stateData)
    lstatePub.publish(stateData)


def rfencCB(encrf):
    global rightprevStateData
    stateData = encrf.data - rightprevStateData
    rightprevStateData = encrf.data
    print("R previous ", rightprevStateData, "State", stateData)
    rstatePub.publish(stateData)


def lmotorCB(lpwm):
    global leftPWM
    leftPWM += lpwm.data
    print("Left PWM", leftPWM)
    leftM.run(leftPWM)


def rmotorCB(rpwm):
    global rightPWM
    rightPWM += rpwm.data
    print("Right PWM", rightPWM)
    rightM.run(rightPWM)


rate = rospy.Rate(10)  # 10hz
lfencoderSub = rospy.Subscriber('enc_lf', Float64, lfencCB)
lmotorSub = rospy.Subscriber('lcontrol_effort', Float64, lmotorCB)
lstatePub = rospy.Publisher('lstate', Float64, queue_size=10)
rfencoderSub = rospy.Subscriber('enc_rf', Float64, rfencCB)
rmotorSub = rospy.Subscriber('rcontrol_effort', Float64, rmotorCB)
rstatePub = rospy.Publisher('rstate', Float64, queue_size=10)

rospy.spin()
rate.sleep()
