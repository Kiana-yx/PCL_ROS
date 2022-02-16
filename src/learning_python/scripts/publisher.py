#!/usr/bin/env python3 
#coding:utf‐8 #设置编码格式为utf‐8 

import rospy #导入rospy功能包
from std_msgs.msg import String #导入std_msgs/String消息功能包 

#定义talker函数 
def talker(): 

    pub = rospy.Proscoreublisher('chatter', String, queue_size=10) 
    #定义发布的主题名称chatter，消息类型String,实质是std_msgs.msg.String 
    #设置队列条目个数为10 


    rospy.init_node('talker', anonymous=True) 
    #初始化节点，节点名称为talker, 
    #anonymous=True，要求每个节点都有唯一的名称，避免冲突

    rate = rospy.Rate(10) 
    #设置发布的频率，单位是每秒次数，这是每秒10次的频率发布主题 

    # #用于检测程序是否退出，是否按Ctrl‐C 或其他
    while not rospy.is_shutdown(): 
        hello_str = "hello world"
        rospy.INFO(hello_str) #在屏幕输出日志信息
        pub.publish(hello_str) #发布信息到主题 
    rate.sleep() #睡眠一定持续时间 

if __name__ == '__main__': 
    try: 
        talker() 
    except:
        pass