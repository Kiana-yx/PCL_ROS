#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy

face_cascade = cv2.CascadeClassifier( '/home/kiana/catkin_ws/src/camera_pkg/scripts/face.xml' ) 

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        global last_erro
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale( gray )
        for (x, y, w, h) in faces:
            img = cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # roi_gray = gray[y:y + h, x:x + w]
            # roi_color = img[y:y + h, x:x + w]
            # eyes = eye_cascade.detectMultiScale(roi_gray)
            # for (ex, ey, ew, eh) in eyes:
            #     cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', image)
        # cv2.waitKey(1)
        # cv2.destroyAllWindows()

def fase_follow():
	# ROS节点初始化
    rospy.init_node('opencv', anonymous=True)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化geometry_msgs::Twist类型的消息
        follower = Follower()
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        fase_follow()
    except rospy.ROSInterruptException:
        pass
