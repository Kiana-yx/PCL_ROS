#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

face_cascade = cv2.CascadeClassifier( '/home/kiana/catkin_ws/src/camera_pkg/scripts/face.xml' ) 

def findBallByColor(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # 在 HSV 颜色空间中要比在 BGR 空间中更容易表示一个特定颜
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 设定蓝色的阈值
    lower_blue = np.array([0, 47, 118])
    upper_blue = np.array([68, 159, 228])

    # 根据阈值构建掩模
    mask_B = cv2.inRange(hsv, lower_blue, upper_blue)    
    mask = mask_B

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    # 开操作 + 闭操作
    ret1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(ret1, cv2.MORPH_CLOSE, kernel, iterations=10)

    
    # 对原图像和掩模进行位运算
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 查找轮廓
    # 下面的代码查找包围框，并绘制
    x, y, w, h = 0, 0, 0, 0
    for cnt in contours:
        area_ball = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        targetPos_x = int(x + w / 2)
        targetPos_y = int(y + h / 2)
        frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    # 显示图像
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('ball_follow', anonymous=True)
    bridge = cv_bridge.CvBridge()
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, findBallByColor)
    rospy.spin()
