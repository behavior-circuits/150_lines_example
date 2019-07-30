#!/usr/bin/env python
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rospy

pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

def cat_avo(msg):
	output = Twist()
	output.linear.x = 1
	output.angular.z = -(0.1/(msg.x*msg.y))
	pub.publish(output)


if __name__ == '__main__':
	try:
		rospy.init_node("cat_avo")
		cat_sub = rospy.Subscriber('/catpose', Point, cat_avo)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- ERROR! ---------")
