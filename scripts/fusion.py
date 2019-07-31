#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class Fusion:
	def __init__(self):
		self.cat       = [0,0]
		self.blue      = [0,0]
		self.orange    = [0,0]
		self.cheese    = [0,0]
		self.collision = [0,0]
		self.pub                     = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		self.cat_avoidance_sub       = rospy.Subscriber(sys.argv[1], Point, self.store_cat)
		self.blue_homing_sub         = rospy.Subscriber(sys.argv[2], Point, self.store_blue)
		self.orange_homing_sub       = rospy.Subscriber(sys.argv[3], Point, self.store_orange)
		self.cheese_homing_sub       = rospy.Subscriber(sys.argv[4], Point, self.store_cheese)
		self.collision_avoidance_sub = rospy.Subscriber(sys.argv[5], Point, self.store_collision)
		while not rospy.is_shutdown():
			self.fusion()


	def store_cat(self,msg):
		self.cat       = move(msg,-0.1,-1,-1)
	def store_blue(self,msg):
		self.blue      = move(msg,0.3,0,1)
	def store_orange(self,msg):
		self.orange    = move(msg,0.3,0,1)
	def store_cheese(self,msg):
		self.cheese    = move(msg,0.3,0,1)
	def store_collision(self,msg):
		self.collision = move(msg,-0.1,-1,-1)

		
	def fusion(self):
		#print('collision', self.collision, 'cheese', self.cheese)
		#rospy.loginfo(self.cat)
		cmd_vel = Twist()
		cmd_vel.linear.x = 1
		#cmd_vel.angular.z = invoke_gate(self.cat[1] , self.orange[1] )
		cmd_vel.angular.z = invoke_gate( cmd_vel.angular.z, self.cat[1] )
		cmd_vel.angular.z = or_gate(cmd_vel.angular.z, self.cheese[1])
		cmd_vel.angular.z = prevail_gate(cmd_vel.angular.z, self.collision[1])
		self.pub.publish(cmd_vel)
		

def move(msg,c1,c2,c3):
	if msg.x != 0 and msg.y != 0 :
	    return np.array([1,c1*msg.x**c2*msg.y**c3])
        else:
            return np.array([1,0])


	
def and_gate(x, y):
        a = 2.28466
        b = -0.89817
        if x == 0 and y == 0:
            return 0
        else:
            return x*(1-np.exp(-((a*y**2+b*x*y)/(x**2+y**2))))+ y*(np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))

def or_gate(x, y):
        a = 1.02889
        b = 0.3574
        if x == 0 and y == 0:
            return 0
        else:
            return x*(np.exp(-((a*y**2+b*x*y)/(x**2+y**2)))) + y*(np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))

def invoke_gate(x, y):
        return and_gate(or_gate(x, y),x)

def prevail_gate(x, y):
        return or_gate(x, or_gate(x, y))

if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")
