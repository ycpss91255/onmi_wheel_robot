#!/usr/bin/env python3
#-*-coding:UTF-8 -*-

from __future__ import print_function

import rospy
import cmath
import numpy as np
from geometry_msgs.msg import Pose2D

from std_msgs.msg import Float64


class Kinematics(object):
    def __init__(self):
        self.L = 1

        self.vector = [0.0, 0.0, 0.0]
        self.enc = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]

        rospy.Subscriber("cmd_motionvector", Pose2D, self.ReadVector)
        # rospy.Subscriber("encoder",Enc,self.ReadEnc)
        self.pub_left_wheel_vel = rospy.Publisher(
            "/omni3robot/left_wheel/command", Float64, queue_size=10)
        self.pub_back_wheel_vel = rospy.Publisher(
            "/omni3robot/back_wheel/command", Float64, queue_size=10)
        self.pub_rigiht_wheel_vel = rospy.Publisher(
            "/omni3robot/rigiht_wheel/command", Float64, queue_size=10)

    # topic function
    def ReadVector(self, data):
        self.vector = [data.x, data.y, data.theta]

    def ReadEnc(self, data):
        self.enc = [data.wheel1_enc, data.wheel2_enc, data.wheel3_enc]
    # Speed function
    def PubSpeed(self):
        print("send")
        self.pub_left_wheel_vel.publish(self.vel[0])
        self.pub_back_wheel_vel.publish(self.vel[1])
        self.pub_rigiht_wheel_vel.publish(self.vel[2])

    def PubStop(self):
        self.pub_left_wheel_vel.publish(0)
        self.pub_back_wheel_vel.publish(0)
        self.pub_rigiht_wheel_vel.publish(0)

    def printf(self):
        print(self.vector)

    # def ForwardKinematic(self):


    def InverseKinematic(self):
        self.vel[0] =  self.L * self.vector[2] - self.vector[0] / 2 + 3 ** 0.5 * self.vector[1] / 2
        self.vel[1] =  self.L * self.vector[2] + self.vector[0]
        self.vel[2] =  self.L * self.vector[2] - self.vector[0] / 2 - 3 ** 0.5 * self.vector[1] / 2
        # print(self.vel)
        self.PubSpeed()

def main():
    rospy.init_node('kinematics')
    rate = rospy.Rate(10)  # 10hz
    kinematics = Kinematics()
    try:
        while not rospy.is_shutdown():
            # kinematics.PubSpeed(10, 10, 10)
            kinematics.InverseKinematic()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
