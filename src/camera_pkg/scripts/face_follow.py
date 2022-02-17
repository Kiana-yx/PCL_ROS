#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy

face_cascade = cv2.CascadeClassifier( '/home/kiana/catkin_ws/src/camera_pkg/scripts/face.xml' ) 

def image_callback(msg):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
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
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('opencv', anonymous=True)
    bridge = cv_bridge.CvBridge()
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()