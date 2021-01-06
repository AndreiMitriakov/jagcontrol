#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray

class Middleware:
    def __init__(self):
        rospy.init_node("middleware")
        rospy.Subscriber('/jaguar/arm_cmd', Float32MultiArray, self.move_arm, queue_size=1)

    def move_arm(self, msg):
        joint1, joint2 = msg.data

    def move_flippers(self, msg):
        front, rear = msg.data

    def move_base(self, msg):
        lin, ang = msg.data



if __name__ == "__main__":
    Middleware()