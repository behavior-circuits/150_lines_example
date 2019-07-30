#!/usr/bin/env python
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rospy



def cheese_home(msg):
	output = Twist()
	output.linear.x = 1
	output.angular.z = sys.argv[2]*msg.y
	pub.publish(output)


if __name__ == '__main__':
	try:
		rospy.init_node("homing")
                pub     = rospy.Publisher(sys.argv[1], Twist, queue_size=1)
		cat_sub = rospy.Subscriber(sys.argv[0], Point, cheese_home)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("homing type node not working")
