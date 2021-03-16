#! /usr/bin/env python
# odometry node


import rospy
import tf
import math
from math import sin, cos, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class odometry:
    ''' Node to oversee odometry information.
        For use with base_controller and PID.
        Publishes motor state data to PID via encoder call backs.
        Todo: incorporate Rviz JointState nav_msgs odometry and tf
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

        self.wheels = rospy.Publisher('Terence', JointState, queue_size=10)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self._lstatePub = rospy.Publisher('lstate', Float64, queue_size=10)
        self._rstatePub = rospy.Publisher('rstate', Float64, queue_size=10)

        rospy.loginfo("checking for Robot Parameters")
        robotParams = rospy.search_param('/Robot name')
        if robotParams:
            name = rospy.get_param('/Robot name')
            rospy.loginfo("Robot Parameters loaded for : %s", name)
        else:
            rospy.logwarn("Robot Parameters not loaded")
        # Retrieve Parameters for physical properties of the robot.
        self._leftMaxRPM = rospy.get_param('/leftMaxRPM')
        self._rightMaxRPM = rospy.get_param('/rightMaxRPM')
        self._wheel_diameter = rospy.get_param('/wheel_diameter')
        self._wheel_base = rospy.get_param('/wheel_base')
        self._leftTPR = rospy.get_param('/leftTicksPerRotation')
        self._rightTPR = rospy.get_param('/rightTicksPerRotation')

        rospy.loginfo("Started Odometry Node")

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

    def moveWheels(self):
        left_wheel_pos = (2 * pi/self._leftTPR) * self._lstate
        right_wheel_pos = (2 * pi/self._rightTPR) * self._rstate

        if left_wheel_pos > pi:
            left_wheel_pos = -pi
        if left_wheel_pos < -pi:
            left_wheel_pos = pi
        if right_wheel_pos > pi:
            right_wheel_pos = -pi
        if right_wheel_pos < -pi:
            right_wheel_pos = pi

        self.Terence.position[0] = left_wheel_pos
        self.Terence.position[1] = right_wheel_pos
        self.wheels.publish(self.Terence)

    def move_robot(self):
        x = 0.0
        y = 0.0
        th = 0.0

        vx = 0.0
        vy = 0.0
        vth = 0.0

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            lvx = (2 * pi/self._leftTPR) * self._lstate
            rvx = (2 * pi/self._rightTPR) * self._rstate
            vx = (lvx + rvx) / 2
            vth = (lvx - rvx) / self._wheel_base
            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th)) #* dt
            delta_y = (vx * sin(th)) #* dt
            delta_th = vth #* dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.odom_pub.publish(odom)

            last_time = current_time
            # self.moveWheels()
            rate.sleep()


def main():
    rospy.init_node('Odometry_node')
    rospy.loginfo("Odometry Node Started")
    odom = odometry()
    odom.move_robot()


if __name__ == '__main__':
    main()
