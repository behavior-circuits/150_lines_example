#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

max_distance = 1
weight       = np.array([  0.5,   0.7,   1.0,   1.0,  1.0,  1.0,  0.7,  0.5])
sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
sonar_angles = sonar_angles / 360.0 * 2 * np.pi

def sonar_callback(data):
    sonar_points = data.points
    sonar_ranges = np.zeros(len(sonar_angles))
    for i in range(0, len(sonar_angles)):
        sonar_ranges[i]=np.sqrt(sonar_points[i].x**2+sonar_points.y**2)
    minimum  = np.argmin(sonar_ranges)
    if sonar_ranges[minimum] <= max_distance:
        output   = Point()
        output.x = sonar_ranges[minimum]*weight[minimum]
        output.y = sonar_angles[minimum]
        pub.publish(output)


if __name__=='__main__':
    try:
        rospy.init_node("obstacle_detection")
        pub       = rospy.Publisher("obstacle",Point,queue_size=1)
        obst_sub  = rospy.Subscriber("/RosAria/sonar",PointCloud,sonar_callback)
    except rospy.ROSInterruptException: 
        rospy.loginfo("sonar not working")

