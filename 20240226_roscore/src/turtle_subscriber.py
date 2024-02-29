#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import math

if __name__ == '__main__':
    # 初始化ros节点，命名为sub_TF
    rospy.init_node('sub_TF')
    # 创建tf2监听器，储存在tfBuffer之中
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # 创建发布者，使得该subscriber使用/listener去操作turtle2的模拟键鼠'/turtle2/cmd_vel'
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1000)
    # 设置循环速度
    rate = rospy.Rate(10.0)
    # 当程序运行的时候一直进入循环
    while not rospy.is_shutdown():
        try:
            # 获取turtle1相对于turtle2的坐标信息，turtle1和turtle2均要参与publish是为了获得相对坐标信息
            tfs = tfBuffer.lookup_transform('turtle2', 'turtle1', rospy.Time(0))
            # 根据坐标信息计算速度信息
            twist = Twist()
            twist.linear.x = 0.5 * math.sqrt(math.pow(tfs.transform.translation.x, 2) +
                                             math.pow(tfs.transform.translation.y, 2))
            twist.angular.z = 4 * math.atan2(tfs.transform.translation.y, tfs.transform.translation.x)
            # 发布者发布信息到该subscriber，即turtle2的键盘指令上接收
            pub.publish(twist)
        except tf2_ros.LookupException:
            continue
        rate.sleep()
