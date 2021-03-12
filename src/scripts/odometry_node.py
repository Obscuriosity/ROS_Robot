#! /usr/bin/env python
# odometry node


import rospy
import math
from std_msgs.msg import Float64


class odometry:
    ''' Node to oversee odometry information.
        For use with base_controller and PID.
        Initially it will just publish motor state data to PID.
        Todo: incorporate Rviz JointState too
    '''

    def __init__(self):
        self._lstate = 0.0
        self._rstate = 0.0
        self._lForstateData = 0.0
        self._lBacstateData = 0.0
        self._rForstateData = 0.0
        self._rBacstateData = 0.0
        self._lForprevStateData = 0.0
        self._lBacprevStateData = 0.0
        self._rForprevStateData = 0.0
        self._rBacprevStateData = 0.0

        self._lfencoderSub = rospy.Subscriber('enc_lf', Float64, self.lfencCB)
        self._lbencoderSub = rospy.Subscriber('enc_lb', Float64, self.lbencCB)
        self._rfencoderSub = rospy.Subscriber('enc_rf', Float64, self.rfencCB)
        self._rbencoderSub = rospy.Subscriber('enc_rb', Float64, self.rbencCB)

        self._lstatePub = rospy.Publisher('lstate', Float64, queue_size=10)
        self._rstatePub = rospy.Publisher('rstate', Float64, queue_size=10)

    def lfencCB(self, enclf):
        self._lForstateData = enclf.data - self._lForprevStateData
        self._lForprevStateData = enclf.data
        # print("----------Left Ticks FOR", self._lForstateData)
        self._lstate = self._lForstateData - self._lBacstateData
        self._lstatePub.publish(self._lstate)

    def lbencCB(self, enclb):
        self._lBacstateData = enclb.data - self._lBacprevStateData
        self._lBacprevStateData = enclb.data
        # print("----------Left Ticks BAC", self._lBacstateData)

    def rfencCB(self, encrf):
        self._rForstateData = encrf.data - self._rForprevStateData
        self._rForprevStateData = encrf.data
        # print("---------Right Ticks FOR", self._rForstateData)
        self._rstate = self._rForstateData - self._rBacstateData
        self._rstatePub.publish(self._rstate)

    def rbencCB(self, encrb):
        self._rBacstateData = encrb.data - self._rBacprevStateData
        self._rBacprevStateData = encrb.data
        # print("---------Right Ticks BAC", self._rBacstateData)


def main():
    rospy.init_node('Odometry_node')
    odometry()
    rospy.spin()


if __name__ == '__main__':
    main()
