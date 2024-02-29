#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 导入必要的库
import rospy
import tf2_ros
import tf_conversions
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

# 乌龟名称全局变量
turtle_name = ''
# Pose的回调函数
def do_pose(pose):
    # 创建TF广播器
    broadcaster = tf2_ros.TransformBroadcaster()
    # 创建TransformStamped消息，并填充数据
    tfs = TransformStamped()
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = 'world'
    tfs.child_frame_id = turtle_name
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0.0
    # 创造四元数，设置旋转
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)
    tfs.transform.rotation.x = q[0]
    tfs.transform.rotation.y = q[1]
    tfs.transform.rotation.z = q[2]
    tfs.transform.rotation.w = q[3]
    # 发布TF变换
    broadcaster.sendTransform(tfs)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('pub_TF')
    # 获取当前挂载了publisher的乌龟名称，并付给全局变量turtle_name
    turtle_name = rospy.get_param('~turtle')
    rospy.loginfo("乌龟 %s 坐标发送启动", turtle_name)
    # 创建ROS订阅者，publisher将/pose的信息broadcast出来
    sub = rospy.Subscriber(turtle_name + '/pose', Pose, do_pose)
    rospy.spin()
