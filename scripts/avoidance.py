#!/usr/bin/env python
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rospy


def cat_avo(msg):
	output = Twist()
	output.linear.x = 1
	output.angular.z = -(0.1/(msg.x*msg.y))
	pub.publish(output)


if __name__ == '__main__':
	try:
		rospy.init_node("cat_avo")
                pub     = rospy.Publisher(sys.argv[2], Twist, queue_size=1)
		cat_sub = rospy.Subscriber(sys.argv[1], Point, cat_avo)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("cat avoidance not working")
