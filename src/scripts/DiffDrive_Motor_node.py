#! /usr/bin/env python
# create diff drive class

import rospy
import math
from DCMotor import DCMotor as DCM
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class diffDrive:
    ''' Differential drive node for use with DCMotor script

        Subscribe to encoder readings from Arduino:
        enc_lf, enc_lb, enc_rf, enc_rb for state readings.
        Subscribe to control_effort for PID control of motors.
        Subscribe to /cmd_vel to accept twist messages.
        Subscribe to /joy to accept xbox joxstick input.

        Publish state readings to PID package.
        Publish pwm to motors.

        Parameters-
        name: str
    '''

    def __init__(self, lfPin, lbPin, rfPin, rbPin,
                 wheel_diameter=0.08, wheel_base=0.196,
                 leftMax_RPM=107, rightMaxRPM=107,
                 leftTicksPerRotation=990, rightTicksPerRotation=990):

        #  Physical properties of the differential motor pair:
        self._leftMaxRPM = leftMax_RPM
        self._rightMaxRPM = rightMaxRPM
        self._wheel_diameter = wheel_diameter
        self._wheel_base = wheel_base
        self._leftTPR = leftTicksPerRotation
        self._rightTPR = rightTicksPerRotation
        self._leftWheel = DCM(lfPin, lbPin)
        self._rightWheel = DCM(rfPin, rbPin)

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
        self._leftPWM = 0.0
        self._rightPWM = 0.0

        self.speed = 0.0
        self.spin = 0.0

        self._lfencoderSub = rospy.Subscriber('enc_lf', Float64, self.lfencCB)
        self._lbencoderSub = rospy.Subscriber('enc_lb', Float64, self.lbencCB)
        self._rfencoderSub = rospy.Subscriber('enc_rf', Float64, self.rfencCB)
        self._rbencoderSub = rospy.Subscriber('enc_rb', Float64, self.rbencCB)
        self._lmotorSub = rospy.Subscriber('lcontrol_effort', Float64, self.lmotorCB)
        self._rmotorSub = rospy.Subscriber('rcontrol_effort', Float64, self.rmotorCB)
        self._cmd_velSub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_CB)
        self._joySub = rospy.Subscriber('joy', Joy, self._joyCB)

        self._lsetpointPub = rospy.Publisher('lsetpoint', Float64, queue_size=10)
        self._rsetpointPub = rospy.Publisher('rsetpoint', Float64, queue_size=10)
        self._lstatePub = rospy.Publisher('lstate', Float64, queue_size=10)
        self._rstatePub = rospy.Publisher('rstate', Float64, queue_size=10)

    def _cmd_vel_CB(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()

    def _joyCB(self, msg):
        '''Translate XBox buttons into speed and spin

        Just use the left joystick (for now):
        LSB left/right  axes[0]     +1 (left) to -1 (right)
        LSB up/down     axes[1]     +1 (up) to -1 (back)
        LB              buttons[5]  1 pressed, 0 otherwise
        '''

        if abs(msg.axes[0]) > 0.10:
            self.spin = msg.axes[0]
        else:
            self.spin = 0.0

        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        if msg.buttons[5] == 1:
            self.speed = 0
            self.spin = 0

        self._set_motor_speeds()

    def lfencCB(self, enclf):
        self._lForstateData = enclf.data - self._lForprevStateData
        self._lForprevStateData = enclf.data
        print("----------Left Ticks FOR", self._lForstateData)
        self._lstate = self._lForstateData - self._lBacstateData
        self._lstatePub.publish(self._lstate)

    def lbencCB(self, enclb):
        self._lBacstateData = enclb.data - self._lBacprevStateData
        self._lBacprevStateData = enclb.data
        print("----------Left Ticks BAC", self._lBacstateData)

    def rfencCB(self, encrf):
        self._rForstateData = encrf.data - self._rForprevStateData
        self._rForprevStateData = encrf.data
        print("---------Right Ticks FOR", self._rForstateData)
        self._rstate = self._rForstateData - self._rBacstateData
        self._rstatePub.publish(self._rstate)

    def rbencCB(self, encrb):
        self._rBacstateData = encrb.data - self._rBacprevStateData
        self._rBacprevStateData = encrb.data
        print("---------Right Ticks BAC", self._rBacstateData)

    def lmotorCB(self, lpwm):
        self._leftPWM += lpwm.data
        print("Left PWM", self._leftPWM)
        self._leftPWM = max(min(self._leftPWM, 100), -100)
        self._leftWheel.run(self._leftPWM)

    def rmotorCB(self, rpwm):
        self._rightPWM += rpwm.data
        print("Right PWM", self._rightPWM)
        self._rightPWM = max(min(self._rightPWM, 100), -100)
        self._rightWheel.run(self._rightPWM)

    def max_speed(self):
        '''Speed in meters per second at maximum RPM'''
        rpm = (self._leftMaxRPM + self._rightMaxRPM) / 2.0
        mps = rpm * math.pi * self._wheel_diameter / 60.0
        return mps

    def max_twist(self):
        '''Rotation in radians per second at maximum RPM'''
        return self.max_speed() / self._wheel_diameter

    def _set_motor_speeds(self):
        if self.speed == 0 and self.spin == 0:
            leftSetpoint = 0
            rightSetpoint = 0
            self._leftWheel.stop()
            self._rightWheel.stop()
        else:
            #
            # Figure out the speed of each wheel based on spin: each wheel covers
            # self._wheel_base meters in one radian, so the target speed for each wheel
            # in meters per sec is spin (radians/sec) times wheel_base divided by
            # wheel_diameter
            #
            right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
            left_twist_mps = -1.0 * self.spin * self._wheel_base / self._wheel_diameter
            #
            # Now add in forward motion.
            #
            left_mps = self.speed + left_twist_mps
            right_mps = self.speed + right_twist_mps
            #
            # Convert meters/sec into RPM: for each revolution, a wheel travels pi * diameter
            # meters, and each minute has 60 seconds.
            #
            left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
            right_target_rpm = (right_mps * 60.0) / (math.pi * self._wheel_diameter)
            #
            # convert rpm to ticks per interval: interval is .1 seconds
            # 1 rotation is 990ticks
            #
            leftSetpoint = (left_target_rpm/600) * self._leftTPR
            rightSetpoint = (right_target_rpm/600) * self._rightTPR
            print("leftSetpoint ", leftSetpoint, " rightSetpoint ", rightSetpoint)
        #
        # Publish setpoints
        #
        self._lsetpointPub.publish(leftSetpoint)
        self._rsetpointPub.publish(rightSetpoint)


def main():
    rospy.init_node('motorControl')
    diffDrive(22, 23, 27, 18)
    rospy.spin()


if __name__ == '__main__':
    main()
