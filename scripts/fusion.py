#!/usr/bin/env python
import sys
import rospy
import numpy as np
import behavior_circuits as bc
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class Fusion:
    def __init__(self):
        # array containes cat,blue,orange,cheese,collision
        self.behaviors = np.zeros(5)
        parameters = np.array([[-0.5, -1, 1, -1], [0.3, 0, 1, 1],
                              [0, 0, 1, 1], [0.8, 0, 1, 1], [-0.5, -1, 1, -1]])
        for i in range(5):
            rospy.Subscriber(sys.argv[i+1], Point, self.generate_behavior,
                             (i, parameters[i, 0], parameters[i, 1], parameters[i, 2], parameters[i, 3]))
        self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        while not rospy.is_shutdown():
            self.fusion()

    def generate_behavior(self, msg, args):
        if msg.x != 0 and msg.y != 0:
            self.behaviors[args[0]] = bc.and(
                bc.sigmoid(msg.x**args[2], args[1], 0), bc.sigmoid(msg.y**args[4], args[3]))
        else:
            self.behaviors[args[0]] = 0

    def fusion(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 1
        cmd_vel.angular.z = bc.prevail(bc.or(bc.invoke(
            self.behaviors[2], self.behaviors[0]), self.behaviors[3]), self.behaviors[4])
        self.pub.publish(cmd_vel)


if __name__ == '__main__':
    try:
        rospy.init_node("fusion")
        fus = Fusion()
    except rospy.ROSInterruptException:
        rospy.loginfo("---------- FUSION-ERRbc.or! ---------")
