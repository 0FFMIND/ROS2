#!/usr/bin/env python3
import rospy  # 这是ROS Python客户端的库，提供了编写ROS节点的功能
import sys  # 提供系统相关的功能和变量
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3  # 引入三种消息类型，用于表示机器人的位置和变换
from nav_msgs.msg import Odometry  # 导入Odometer消息类型，用来处理里程计信息
from tf2_ros import TransformBroadcaster  # 这个类提供简单的方式发布坐标变换

# 定义PoseSetter类，用来设置固定的位置
class PoseSetter:
    # 其构造函数接受一个参数myargv
    def __init__(self, myargv):
        self.pose = PoseStamped()  # PoseStamped用于表示一个带有时间戳的目标点位置，常用于表示起始坐标点/机器人需要前往的目标点
        self.name = myargv[0]  # 用来设置机器人名字
        self.multi_mode = myargv[1]  # 用来设置多机器人模式
        self.num = myargv[2]  # 用来设置机器人序号
        self.pose.pose.position = self.robot_formation[self.multi_mode][self.num]

    # 创建了一个嵌套字典，通过.launch文件传入的键值去寻找对应坐标Vector3的值最后赋给PoseStamped()中的position,使其得到xyz的值
    robot_formation = {
        'column': {'1': Vector3(0., 0., 0.), '2': Vector3(-0.8, 0., 0.), '3': Vector3(-1.6, 0., 0.)},
        'row': {'1': Vector3(0., 0., 0.), '2': Vector3(0., -0.8, 0.), '3': Vector3(0., -1.6, 0.)},
        'triangle': {'1': Vector3(0., 0., 0.), '2': Vector3(-0.8, 0.6, 0.), '3': Vector3(-0.8, -0.6, 0.)},
    }

# 定义tf2_Broadcaster类，用来广播TF变换
class tf2_Broadcaster:
    def __init__(self):
        # 在tf2.launch运行的时候虽然传入了三个参数arg，但最前面会存在一个remapping argument，尽管传入三个参数，实际上.py脚本接收到rospy.myargv会有四个
        if len(rospy.myargv()) != 4:
            rospy.logerr("传入的参数不等于4")  # 若传入的argv不为3,则出现报错消息
            sys.exit(1)  # 退出程序
        # 创建一个subscriber的对象来订阅.../odom的话题，其接收消息的类型为Odometer，回调函数为cb_pose
        self.sub = rospy.Subscriber(rospy.myargv()[1] + "/odom", Odometry, callback=self.cb_pose)
        self.poseSetter = PoseSetter(rospy.myargv()[1:4])  # 将外部.launch文件输入的三个参数传入PoseSetter
    def cb_pose(self, pose):
        # 定义了名为cb_pose的回调函数，用于处理接收到的里程计消息
        broadcaster = TransformBroadcaster()  # 调用tf2_ros功能包实现简单的坐标广播
        tfs = TransformStamped()  # 创建tfs对象的格式用来广播信息，需要设置动态坐标
        tfs.header.frame_id = "world"  # 设置变换信息的父坐标系
        tfs.header.stamp = rospy.Time.now()  # 设置变换的时间戳为当前时间，不然在Rviz中会出现错误
        tfs.child_frame_id = self.poseSetter.name  # 设置变换的子坐标系为小机器人
        # 计算父坐标系对子坐标系的偏差
        tfs.transform.translation.x = pose.pose.pose.position.x - self.poseSetter.pose.pose.position.x
        tfs.transform.translation.y = pose.pose.pose.position.y - self.poseSetter.pose.pose.position.y
        tfs.transform.translation.z = 0
        # 直接将订阅到的里程计消息的方向赋给旋转的方向
        tfs.transform.rotation = pose.pose.pose.orientation
        broadcaster.sendTransform(tfs)

if __name__ == '__main__':
    # 使用rospy生成一个名为/dynamic_tf_pub的节点，并允许同名节点的存在，通过给予不同空间保证了其唯一性
    rospy.init_node("dynamic_tf_pub", anonymous=True)
    # 创建实例
    tracker = tf2_Broadcaster()
    try:
        # 这个函数用来保证挂载该脚本的程序一直运行直到节点被关闭
        rospy.spin()
    except Exception as e:
        # 如果在spin的过程中遇到异常进行异常捕捉
        rospy.logerr(e)
