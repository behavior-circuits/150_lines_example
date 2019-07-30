#!/usr/bin/env python

import sys
import time
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool




mask2 = cv2.imread('/home/tobias/catkin_ws/src/maus/scripts/mask.jpg',0)
if not mask2.any() :
	print 'cant load mask'
realWidth_cheese = 24
realWidth_cat = 24
F_x = 500
activation_threshold=0.1
alpha=2.
col_vel = Twist()
current_vel = Odometry()
bridge = CvBridge()

rospy.init_node('main_node')


pubCheese = rospy.Publisher('cheesepose', Point, queue_size=1)
cavo_vel_pub = rospy.Publisher('collision_avoidance_vel', Twist, queue_size=1)
pubCurt = rospy.Publisher('curtain', Point, queue_size=1)
pubCat = rospy.Publisher('catpose', Point, queue_size=1)
fusion_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

cheese_lower = 0
cheese_upper = 0
cheese_boundaries = [([50,60,0],[80,255,100])]
for (lower, upper) in cheese_boundaries:
	cheese_lower = np.array(cheese_lower, dtype = "uint8")
	cheese_upper = np.array(cheese_upper, dtype = "uint8")

cat_lower = 0
cat_upper = 0
cat_boundaries = [([0,120,30],[22,255,125])]
for (lower, upper) in cat_boundaries:
	cat_lower = np.array(cat_lower, dtype = "uint8")
	cat_upper = np.array(cat_upper, dtype = "uint8")

curt_lower = 0
curt_upper = 0
curt_boundaries = [([50,60,0],[80,255,100]),([50,60,0],[80,255,100])]
for (lower, upper) in curt_boundaries:
	curt_lower = np.array(curt_lower, dtype = "uint8")
	curt_upper = np.array(curt_upper, dtype = "uint8")


def F_x_call(data):
	F_x = data.K[0]
	print F_x
	cam_info_sub.unregister()

def detect_cheese(ros_img):
	cv_img = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, cheese_lower, cheese_upper)
	bitwise = cv2.bitwise_and(hsv_img, hsv_img, mask = mask) 
	bitwise = cv2.bitwise_and(bitwise, bitwise, mask = mask2)
	bitwise = cv2.medianBlur(bitwise,9)
	edged = cv2.Canny(bitwise, 0 ,50 , L2gradient=True, )
	blur = cv2.GaussianBlur(edged,(3,3),0)
	ret,thresh = cv2.threshold(blur,127,255,0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	x = []
	y = []
	w = []
	h = []
	if len(contours) > 0:
		for a in contours:
			xtemp,ytemp,wtemp,htemp = cv.boundingRect(a)
			x.append(xtemp)
			y.append(ytemp)
		w = (max(x)-min(x))
		h = (max(y)-min(y)) 
		#cv.rectangle(cv_img,(min(x),max(y)),(min(x)+w,max(y)-h),(0,255,0),2)
		pub = Point()
		xsize = w
		ysize = h
		xpos =  min(x)
		ypos =  max(y)
		if xsize > 0 :
			
			pub.z = computeDistance(xsize)
			pixz = (realWidth * F_x)/pub.z
			if xpos >= 320:
				relxpos = xpos - 320 - (xsize/2)
			else:
				relxpos = -(320 - xpos) + (xsize/2)
			pub.y = computeAngleFromPoint(relxpos,pixz)
			pub.x = relxpos*w*h
		
		pubCheese.publish(pub)
	else:
		pub = Point()
		return pub


def detect_curt(ros_img):
	cv_img = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, curt_lower, curt_upper)
	bitwise = cv2.bitwise_and(hsv_img, hsv_img, mask = mask) 
	bitwise = cv2.bitwise_and(bitwise, bitwise, mask = mask2)
	bitwise = cv2.medianBlur(bitwise,9)
	edged = cv2.Canny(bitwise, 0 ,10 , L2gradient=True, )
	blur = cv2.GaussianBlur(edged,(3,3),0)
	ret,thresh = cv2.threshold(blur,127,255,0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	x = []
	y = []
	w = []
	h = []
	if len(contours) > 0:
		for a in contours:
			xtemp,ytemp,wtemp,htemp = cv.boundingRect(a)
			x.append(xtemp)
			y.append(ytemp)
		w = (max(x)-min(x))
		h = (max(y)-min(y)) 
		#cv.rectangle(cv_img,(min(x),max(y)),(min(x)+w,max(y)-h),(0,255,0),2)
		pub = Point()
		xsize = w
		ysize = h
		
		if xsize > 0 :
			pub.z = computeDistance(xsize)
			pixz = (realWidth_cat * F_x)/pub.z
			if xpos >= 320:
				relxpos = xpos - 320 - (xsize/2)
			else:
				relxpos = -(320 - xpos) + (xsize/2)
		pub.x = (w*h)*relxpos
		pub.y = computeAngleFromPoint(relxpos,pixz)
		pubCurt.publish(pub)
	else:
		pub = Point()
		return pub 
	
def detect_cat(ros_img):
	cv_img = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, cat_lower, cat_upper)
	bitwise = cv2.bitwise_and(hsv_img, hsv_img, mask = mask) 
	bitwise = cv2.bitwise_and(bitwise, bitwise, mask = mask2)
	bitwise = cv2.medianBlur(bitwise,9)
	edged = cv2.Canny(bitwise, 0 ,50 , L2gradient=True, )
	blur = cv2.GaussianBlur(edged,(3,3),0)
	ret,thresh = cv2.threshold(blur,127,255,0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	x = []
	y = []
	w = []
	h = []
	if len(contours) > 0:
		for a in contours:
			xtemp,ytemp,wtemp,htemp = cv.boundingRect(a)
			x.append(xtemp)
			y.append(ytemp)
		w = (max(x)-min(x))
		h = (max(y)-min(y)) 
		#cv.rectangle(cv_img,(min(x),max(y)),(min(x)+w,max(y)-h),(0,255,0),2)
		pub = Point()
		xsize = w
		ysize = h
		xpos =  min(x)
		ypos =  max(y)
		if xsize > 0 :
			pub.z = computeDistance(xsize)
			pixz = (realWidth_cat * F_x)/pub.z
			if xpos >= 320:
				relxpos = xpos - 320 - (xsize/2)
			else:
				relxpos = -(320 - xpos) + (xsize/2)
			pub.y = computeAngleFromPoint(relxpos,pixz)
			pub.x = -(w*h)/relxpos * 1
		pubCat.publish(pub)rho/4*
	else:
		pub = Point()
		return pub

def computeAngleFromPoint(x,z):
	angle = np.arcsin(x/np.sqrt(x**2+z**2))
	return angle

def img_call(data):
	cheese = detect_cheese(data)
	cat = detect_cat(data)
	curt = detect_curt(data)
	fusion(col_vel, cat, cheese, curt )

def avoid_collision(pointCloud):
		new_cmd_vel=Twist()
		scan_data = pointCloud.points
		total_force = calculate_force(scan_data)
		if total_force[1] != 0:
			# If forward directoin is too strong, decrease it
			if np.abs(total_force[0]/total_force[1]) < 0.25:
				new_cmd_vel.linear.x=current_cmd_vel.linear.x*(1-total_force[0]/np.linalg.norm(total_force))
			# Only do collision avoidance, if we have a positive forward movement
			if current_vel.twist.twist.linear.x>=0 :
				# adaptive angular velocity is the angle of the force vector time the current forward velocity times a scaling factor
				new_cmd_vel.angular.z = alpha * np.arctan2(-total_force[1] , total_force[0])
				if np.abs(new_cmd_vel.angular.z) < activation_threshold:
					new_cmd_vel.angular.z=0
		else:
			new_cmd_vel.angular.z=0
		col_vel = new_cmd_vel
		cavo_vel_pub.publish(new_cmd_vel)

def calculate_force(points):
	F=np.array([0.,0.])
	for point in points:
		#TODO: vectorize
		#directions.append(np.arctan2(point.y,point.x ))           
		#distances.append(np.sqrt(point.y**2+point.x**2))
		F+=np.array([point.x,point.y])/(np.sqrt(point.y**2+point.x**2)**2)*np.cos(np.arctan(point.y/point.x ))
	return np.round(F, decimals=0) + 0

def turn_and_drive(current_goal_position):
		turn_and_drive_cmd_vel = Twist()
		turn_and_drive_cmd_vel.angular.z = current_goal_position.y#/90 
		if current_goal_position.z > 0.1:
			turn_and_drive_cmd_vel.linear.x = 1 #vollgas
		else:
			turn_and_drive_cmd_vel.linear.x = 0.025
		return turn_and_drive_cmd_vel



def and_gate(x, y):
        a = 2.28466
        b = -0.89817
        if x == 0 and y == 0:
            return 0
        else:
            return x(1-np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))+ y(np.exp(-((a*x**2+b*x*y)/(x**2+y**2))))

def or_gate(x, y):
        a = 1.02889
        b = 0.3574
        if x == 0 and y == 0:
            return 0
        else:
            return np.exp(-((a*x**2+b*x*y)/(x**2+y**2))) + np.exp(-((a*x**2+b*x*y)/(x**2+y**2)))

def invoke_gate(x, y):
        return and_gate(or_gate(x, y),x)

def prevail_gate(x, y):
        return or_gate(x, or_gate(x, y))


def fusion(col_vel, cat, cheese, curtain):
	vel_cmd = Twist()
	vel_cmd.linear.x = 10

	#vel_cmd.angular.z = invoke_gate(curtain.x, cat.x)
	#vel_cmd.angular.z = and_gate(vel_cmd.angular.z,cheese.x)
	#vel_cmd_goal = turn_and_drive(cheese)
	#vel_cmd.angular.z  = prevail_gate(col_vel.angular.z,vel_cmd.angular.z)
	vel_cmd.angular.z =  cat.x
	print type(vel_cmd)
	fusion_pub.publish(vel_cmd)
	
def pose_call(odom):
	current_vel = odom

if __name__ == '__main__':
	try:
		cam_info_sub = rospy.Subscriber('/usb_cam/camera_info',CameraInfo,F_x_call)
		img_sub =rospy.Subscriber('/usb_cam/image_raw',Image,img_call)
		sonar_sub = rospy.Subscriber('/RosAria/sonar', PointCloud, avoid_collision)
		pose_sub = rospy.Subscriber('/RosAria/pose', Odometry, pose_call)
		

		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- ERROR! ---------")
