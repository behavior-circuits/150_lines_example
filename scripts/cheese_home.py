#!/usr/bin/env python
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rospy

pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

k=1

def cheese_home(msg):
	output = Twist()
	output.linear.x = 1
	output.angular.z = k*msg.y
	pub.publish(output)


if __name__ == '__main__':
	try:
		rospy.init_node("cheese_home")
		cat_sub = rospy.Subscriber('/cheesepose', Point, cheese_home)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- ERROR! ---------")
