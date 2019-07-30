#!/usr/bin/env python
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np


focal_len        = 750
min_size         = 0
max_height       = 200
height_of_object = sys.argv[1]
lower_bound      = np.array([sys.argv[2],sys.argv[3],sys.argv[4]])
upper_bound      = np.array([sys.argv[5],sys.argv[6],sys.argv[7]])
memory           = sys.arg[8]
global last_cheese
last_cheese      = 0

def img_call(ros_img):
	global last_cheese
	cv_img                   = bridge.imgmsg_to_cv2(ros_img,'rgb8')
	image_h, image_w         = cv_img.shape[:2]
	hsv_img                  = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
	mask                     = cv2.inRange(hsv_img, cheese_lower, cheese_upper)
	mask                     = cv2.rectangle(mask,(0,0),(len(mask[0]),max_height),(0,100,0),-1)
	im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	output                   = Point()
        if memory == True:
            output.x                 = 1
            output.y                 = last_cheese

	if len(contours) > 0:
		biggest_countour=max(contours, key = cv2.contourArea)
		if cv2.contourArea(biggest_contour) >= min_size:
			x,y,w,h         = cv2.boundingRect(biggest_contour[0])
			bounding_center     = np.array([x+w/2,y+cheese_height/2])
			output              = Point()
			output.y            = -1*np.arctan((bounding_center[0]-image_w/2.0)/focal_len)
			output.x            = height_of_object*focal_len/(h*np.cos(alpha))
			last_cheese         = output.y
	pubCheese.publish(output)


if __name__ == '__main__':
	try:
		rospy.init_node("vision")
                bridge       = CvBridge()
		cam_info_sub = rospy.Subscriber('/usb_cam/image_raw',Image,img_call)
                pubCheese    = rospy.Publisher(sys.argv[0], Point, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("vision type node not working")
