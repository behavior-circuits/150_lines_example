#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

max_distance = 1
weight = np.array([0.5,   0.7,   1.0,   1.0,  1.0,  1.0,  0.7,  0.5])
sonar_angles = np.array(
    [-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0]) / 360.0 * 2 * np.pi


class Sonar:
    def __init__(self):
        self.hideout = np.zeros(2)
        rospy.Subscriber("/RosAria/sonar", PointCloud, self.sonar_callback)
        rospy.Subscriber("blue_position", Point, self.hideout_callback, 0)
        rospy.Subscriber("orange_position", Point, self.hideout_callback, 1)
        while not rospy.is_shutdown():
            print("still working")

    def hideout_callback(self, data, i):
        if data.x != 0:
            self.hideout[i] = data.x
        else:
            self.hideout[i] = 100000

    def sonar_callback(self, data):
        pub = rospy.Publisher("obstacle_position", Point, queue_size=1)
        sonar_points = data.points
        sonar_ranges = np.zeros(len(sonar_angles))
        for i in range(0, len(sonar_angles)):
            sonar_ranges[i] = np.sqrt(sonar_points[i].x**2+sonar_points[i].y**2) \
                * 2*(1/(1+np.exp(-1*(min(self.hideout))))-0.5)
        minimum = np.argmin(sonar_ranges)
        output = Point()
        if sonar_ranges[minimum] <= max_distance:
            output.x = sonar_ranges[minimum]*weight[minimum]
            output.y = -sonar_angles[minimum]
        pub.publish(output)


if __name__ == '__main__':
    rospy.init_node("obstacle_detection")
    try:
        node = Sonar()
    except rospy.ROSInterruptException:
        rospy.loginfo("sonar not working")
