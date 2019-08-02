#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class Fusion:
	def __init__(self):
                # array containes cat,blue,orange,cheese,collision
                self.behaviors = np.zeros((5,2))
                parameters     =  np.array([[-0.1,-1,-1],[0.3,0,1],[0,0,1],[0.8,0,1],[-0.5,-1,-1]])
                for i in range(1,5):
                    rospy.Subscriber(sys.argv[i], Point, self.generate_behavior,(i,parameters[i,0],parameters[i,1],parameters[i,2]))
		self.pub                     = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		while not rospy.is_shutdown():
			self.fusion()

        def generate_behavior(self,msg,args):
                if msg.x != 0 and msg.y != 0 :
                    self.behaviors[args[0]] = np.array([1,args[1]*msg.x**args[2]*msg.y**args[3]])
                else:
                    self.behaviors[args[0]] = np.array([1,0])

	def fusion(self):
		#print('collision', self.collision, 'cheese', self.cheese)
		cmd_vel = Twist()
		cmd_vel.linear.x = 1
                cmd_vel.angular.z = prevail_gate(or_gate(invoke_gate(self.behaviors[0,1],self.behaviors[2,1]),self.behaviors[3,1]),self.behaviors[4,1])
                print(cmd_vel)
		self.pub.publish(cmd_vel)

	
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
