#! /usr/bin/env python


import rospy
rospy.init_node('synchronizer')

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import message_filters
from sensor_msgs.msg import Image, CameraInfo

assert __name__ == '__main__'




cvBridge   = CvBridge()
fisheye    = rospy.get_param('~fisheye', True)
use_undistort = rospy.get_param('~undistort', False)

def callback(image, camera_info):
	global cvBridge, fisheye, out_width, out_height

	if fisheye:
		K = np.array(camera_info.K).reshape(3, 3)
		D = np.array(camera_info.D)

		image_cv = cvBridge.imgmsg_to_cv2(image, 'mono8')
		if use_undistort:
			image_cv = cv2.undistort(image_cv, K, D)

		image = cvBridge.cv2_to_imgmsg(image_cv, 'mono8')

	stamp1 = image.header.stamp
	stamp2 = camera_info.header.stamp
	stamp  = max(stamp1, stamp2)

	image.header.stamp = stamp
	camera_info.header.stamp  = stamp

	image_pub.publish(image)
	info_pub.publish(camera_info)


image_pub = rospy.Publisher('image_sync', Image, queue_size=10)
info_pub  = rospy.Publisher('camera_info_sync', CameraInfo, queue_size=10)
image_sub = message_filters.Subscriber('image', Image)
info_sub  = message_filters.Subscriber('camera_info', CameraInfo)
ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
ts.registerCallback(callback)


rate = rospy.Rate(10)

try:
	while not rospy.is_shutdown():
		rospy.spin()
		rate.sleep()
except rospy.exceptions.ROSInterruptException:
	pass
