#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

mask2 = cv2.imread('/home/tobias/catkin_ws/src/maus/scripts/mask.jpg',0)
if not mask2.any() :
	print 'cant load mask'
realHeight_cheese = 24
realHeight_cat = 24
realHeight_curt = 100
F_x = 500
bridge = CvBridge()
pubCheese = rospy.Publisher('cheesepose', Point, queue_size=1)
pubCurt = rospy.Publisher('curtain', Point, queue_size=1)
pubCat = rospy.Publisher('catpose', Point, queue_size=1)


min_size               = 0
cheese_lower = np.array([50,60,0])
cheese_upper = np.array([80,255,100])
height_of_cheese       = 0.10


height_of_curtain       = 0.66
cat_lower = np.array([0,120,30])
cat_upper = np.array([22,255,125])
height_of_cat       = 0.18
curt_lower_blue = np.array([50,60,0])
curt_upper_blue = np.array([80,255,100])
curt_lower_orange = np.array([50,60,0])#todo
curt_upper_orange = np.array([80,255,100])#todo
global last_cheese 
last_cheese = 0

def img_call(ros_img):
	global last_cheese
	# detect cheese
	#-----------------------------------------------------------------------------------------------
	cv_img = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	image_h, image_w       = cv_img.shape[:2]
	hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, cheese_lower, cheese_upper)
	bitwise = cv2.bitwise_and(mask, mask, mask = mask2)
	im2, contours, hierarchy = cv2.findContours(bitwise, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#print len(contours)
	output              = Point()
	output.x = 1
	output.y =last_cheese
	if len(contours) > 0:
		contours.sort(key = cv2.contourArea,reverse=True)
		if cv2.contourArea(contours[0]) >= min_size:
			x,y,w,cheese_height = cv2.boundingRect(contours[0])
			bounding_center     = np.array([x+w/2,y+cheese_height/2])
			output              = Point()
			output.y            = -1*np.arctan((bounding_center[0]-image_w/2.0)/F_x)
			output.x            = height_of_cheese/cheese_height*F_x/np.cos(alpha)
			last_cheese = output.y
	pubCheese.publish(output)
	#--------------detect cat---------------------------------------------------
	mask = cv2.inRange(hsv_img, cat_lower, cat_upper)

	bitwise = cv2.bitwise_and(mask, mask, mask = mask2)
	im2, contours, hierarchy = cv2.findContours(bitwise, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print len(contours)
	if len(contours) > 0:
		contours.sort(key = cv2.contourArea,reverse=True)
		if cv2.contourArea(contours[0]) >= min_size:
			x,y,w,cheese_height = cv2.boundingRect(contours[0])
			bounding_center     = np.array([x+w/2,y+cheese_height/2])
			output              = Point()
			output.y            = -1*np.arctan((bounding_center[0]-image_w/2.0)/F_x)
			output.x            = height_of_cat/cheese_height*F_x/np.cos(alpha)
			pubCat.publish(output)


	#--------------curtain---------------------------------------------------------------------------------
	cv_img = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	image_h, image_w       = cv_img.shape[:2]
	hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, curt_lower, curt_upper)
	bitwise = cv2.bitwise_and(mask, mask, mask = mask2)
	im2, contours, hierarchy = cv2.findContours(bitwise, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#print len(contours)
	output = Point()
	output.x = 1
	if len(contours) > 0:
		contours.sort(key = cv2.contourArea,reverse=True)
		if cv2.contourArea(contours[0]) >= min_size:
			#print 'beep'
			x,y,w,curt_height = cv2.boundingRect(contours[0])
			bounding_center     = np.array([x+w/2,y+cheese_height/2])
			output              = Point()
			output.y            = -1*np.arctan((bounding_center[0]-image_w/2.0)/F_x)
			output.x            = height_of_curtain/curt_height*F_x/np.cos(alpha)	
                        pubCheese.publish(output)
		




def F_x_call(data):
	F_x = data.K[0]
	print F_x
	cam_info_sub.unregister()

if __name__ == '__main__':
	try:
		rospy.init_node("vision")
		cam_info_sub = rospy.Subscriber('/usb_cam/camera_info',CameraInfo,F_x_call)
		img_sub =rospy.Subscriber('/usb_cam/image_raw',Image,img_call)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- ERROR! ---------")
