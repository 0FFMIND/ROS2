#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from turtlesim.srv import Spawn

def create_turtle():
    # 初始化ROS节点，命名为create_turtle
    rospy.init_node('create_turtle', anonymous=True)
    # 等待‘/spawn’的Service，当它变得可用
    rospy.wait_for_service('/spawn')
    try:
        # 创建'/spawn'的服务
        spawn_turtle_service = rospy.ServiceProxy('/spawn', Spawn)
        # 对应的请求数据，对应乌龟的名字，生成的坐标位置等
        turtle_name = "turtle2"
        x = 1.0
        y = 2.0
        theta = 3.1415926
        # 发送请求
        response = spawn_turtle_service(x, y, theta, turtle_name)
        # 只调用一次，如果请求成功
        if response.name:
            rospy.loginfo("乌龟 %s 创建成功", response.name)
        else:
            rospy.loginfo("乌龟创建失败")
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败：%s", e)

if __name__ == '__main__':
    try:
        create_turtle()
    except rospy.ROSInterruptException:
        pass
