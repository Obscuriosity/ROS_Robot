#! /usr/bin/env python
# bumper_node.py

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class bumpers:
    '''
    Bumper node.

    Subscribes to bpr messages from arduino serial.
    Publishes cmd_vel for Diff Drive Motor node.
    '''

    def __init__(self):

        self._vel_msg = Twist()

        self._bumperLsub = rospy.Subscriber('bpr_lf', Bool, self.lbprCB)
        self._bumperFsub = rospy.Subscriber('bpr_mf', Bool, self.fbprCB)
        self._bumperRsub = rospy.Subscriber('bpr_rf', Bool, self.rbprCB)
        self._reaction = rospy.Publisher('reaction', Twist, queue_size=10)

    def lbprCB(self, msg):
        if msg.data is True:
            rospy.loginfo("Left bumper hit")
            self._reaction.publish(self.stop())

    def fbprCB(self, msg):
        if msg.data is True:
            rospy.loginfo("Front bumper hit")
            self._reaction.publish(self.stop())

    def rbprCB(self, msg):
        if msg.data is True:
            rospy.loginfo("Right bumper hit")
            self._reaction.publish(self.stop())

    def stop(self):
        self._vel_msg.linear.x = 0
        self._vel_msg.linear.y = 0
        self._vel_msg.linear.z = 0
        self._vel_msg.angular.x = 0
        self._vel_msg.angular.y = 0
        self._vel_msg.angular.z = 0

    def rotateLeft(self):
        self._vel_msg.linear.x = 0
        self._vel_msg.linear.y = 0
        self._vel_msg.linear.z = 0
        self._vel_msg.angular.x = 0
        self._vel_msg.angular.y = 0
        self._vel_msg.angular.z = 0.01


def main():
    rospy.init_node('bumpers')
    bumpers()
    rospy.spin()


if __name__ == '__main__':
    main()
