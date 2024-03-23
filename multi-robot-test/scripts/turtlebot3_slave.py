#!/usr/bin/env python3
import rospy  # 这是ROS Python客户端的库，提供了编写ROS节点的功能
import math  # 提供数学运算
import sys  # 提供系统相关的功能和变量
import tf2_ros  # 用于处理Transform的变换
from sensor_msgs.msg import LaserScan  # 用于接收激光雷达的数据
from geometry_msgs.msg import Twist, TransformStamped  # Twist用于发送速度命令
from nav_msgs.msg import Odometry  # 用于接收和处理位置和姿态信息

# 定义度到弧度，弧度到度的转换常量
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi
# 定义用于索引到激光雷达数据的常量，引用前方，左侧和右侧的距离值
CENTER = 0
LEFT = 1
RIGHT = 2
# 定义机器人线速度和角速度的常量
LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 1.5
# 定义用于控制逻辑中确认机器人的行为的状态常量
GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3

# 定义Drive类，它封装了对机器人的驱动逻辑
class Turtlebot3Drive:
    # 构造函数，接受源坐标系和目标坐标系作为参数
    def __init__(self, source_frame, target_frame):
        # 初始化了一个ROS节点，而anonymous=True表示即使有同名的节点也可以同时运行，因为ROS会自动给节点加上唯一标识
        rospy.init_node('turtlebot3_drive', anonymous=True)
        # 创建变量存储坐标系
        self.source_frame = source_frame
        self.target_frame = target_frame
        # 初始化一个列表用来储存从激光雷达接收到的数据，设置为0，分别对应Center，Left，Right扫描到的数据
        self.scan_data = [0.0, 0.0, 0.0]
        # 创建一个Publisher对象，向source_frame/cmd_vel主题发布Twist类型的数据，用于控制机器人的速度
        self.cmd_vel_pub = rospy.Publisher(f'{source_frame}/cmd_vel', Twist, queue_size=10)
        # 创建两个Subscriber对象，用于订阅激光雷达scan和里程计odom话题，指定消息类型后分配对应的回调函数
        self.laser_scan_sub = rospy.Subscriber(f'{source_frame}/scan', LaserScan, self.laser_scan_msg_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(f'{source_frame}/odom', Odometry, self.odom_callback, queue_size=10)
        # 创建Buffer和Listener的对象，用于监听和缓存坐标变换信息(tf_buffer中储存相对Transform变换的信息)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # 初始化小车避障参数，第一个参数为转向时的转向范围，第二个为机器人前方检查距离，第三个为机器人侧方检查距离
        self.escape_range = 30 * DEG2RAD
        self.check_forward_dist = 0.7
        self.check_side_dist = 0.6
        # 初始化当前姿态角和上一时刻的姿态角，设置为0
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0
        # 初始化小车状态机此时的状态，为等待输入的类型
        self.tb3_state = GET_TB3_DIRECTION
        # 初始化两辆小车坐标帧的变换
        self.robot1_to_robot2 = None

    # 提供给laser_scan_sub的回调函数，对应的更新scan_data的值
    def laser_scan_msg_callback(self, msg):
        # ros标准坐标系是右手坐标系，直线0度为最后面，正转180度为正前方，正转180+30度为正前方左转30度，左侧，同理，180-30度为右侧
        scan_angle = [180, 210, 150]  # 定义一个列表包含三个角度值，0，30，330，代表前方，左侧，右侧
        for num in range(3):  # num表示当前角度的索引，为0,1,2
            # msg.ranges[x]，获取msg中下标为x的测距值
            if math.isinf(msg.ranges[scan_angle[num]]):  # 检测前方是否有无穷远的读数(即没有检测到任何物体)
                self.scan_data[num] = msg.range_max  # 若为无穷远，则返回激光扫描仪的最大距离
            else:  # 当对应扫描的角度检测到距离的时候
                self.scan_data[num] = msg.ranges[scan_angle[num]]  # 将扫描到的距离储存到scan_data对应的num位置上

    # 提供给odom_sub的回调函数，对应的更新tb3_pose的值
    def odom_callback(self, msg):
        # 将msg中的四元数转换为欧拉角的中间变量sin_y
        sin_y = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                       msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        # 将msg中的四元数转换为欧拉角的中间变量cos_y
        cos_y = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                             msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.tb3_pose = math.atan2(sin_y, cos_y)  # 最后使用math库计算并更新当前机器人的偏行角(绕z轴的旋转)

    # 定义一个更新机器人速度的方法，通过创建Twist实例将角速度和线速度Publish到/cmd_vel的节点
    def update_command_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    # 定义一个根据激光雷达扫描的信息改变机器人状态的方法
    def update_tb3_state(self):
        if self.scan_data[CENTER] > self.check_forward_dist:  # 当雷达找到的可以向前距离大于向前距离的阈值
            if self.scan_data[LEFT] < self.check_side_dist:  # 当雷达找到可以向左的距离小于安全边缘距离的时候
                self.prev_tb3_pose = self.tb3_pose  # 将此时小车的转向角存储为pre，意味着接下来的转向角会得到更新
                self.tb3_state = TB3_RIGHT_TURN  # 更新小车状态为向右
            elif self.scan_data[RIGHT] < self.check_side_dist:  # 找到向右的距离小于安全距离时同理
                self.prev_tb3_pose = self.tb3_pose
                self.tb3_state = TB3_LEFT_TURN  # 更新小车状态为向左
            # 左右检查距离都在安全范围内就将状态写为直行，并且不用改变转向角
            else:
                self.tb3_state = TB3_DRIVE_FORWARD
        if self.scan_data[CENTER] < self.check_forward_dist:  # 当雷达找到的可以向前距离小于向前距离的阈值
            self.prev_tb3_pose = self.tb3_pose  # 将此时小车的转向角pose存为先前的转向角
            self.tb3_state = TB3_RIGHT_TURN  # 更新状态为向右(写死了)

    # 用于开启循环的函数
    def control_loop(self):
        rate = rospy.Rate(50)  # 设置循环速度为50Hz
        while not rospy.is_shutdown():  # 在该ros节点没有被关闭的时候一直执行循环
            try:  # 从tf2的缓冲区获得当前robot1与robot2的坐标系变换信息
                rate.sleep()
                self.robot1_to_robot2 = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame,
                                                                        rospy.Time(0), rospy.Duration(1))
                # 计算两个坐标系之间的直线距离和角度信息
                line = math.sqrt(pow(self.robot1_to_robot2.transform.translation.x, 2) +
                                 pow(self.robot1_to_robot2.transform.translation.y, 2))  # 计算两个坐标系之间的直线距离
                angle = math.atan2(self.robot1_to_robot2.transform.translation.y,
                                   self.robot1_to_robot2.transform.translation.x)  # 计算从source_frame到target_frame的角度
                # 进行当前tb3_state的判断从而执行不同的逻辑
                if self.tb3_state == GET_TB3_DIRECTION:
                    self.update_tb3_state()
                elif self.tb3_state == TB3_DRIVE_FORWARD:
                    if line > 2: line = 2  # 当直线距离大于2的时候设置直线距离为2
                    elif line < 0.1: angle = 0  # 当直线距离小于0.1的时候设置角度为0
                    self.update_command_velocity(0.5 * line, 4 * angle)  # 官方文档设置的参数
                    self.tb3_state = GET_TB3_DIRECTION  # 复原
                elif self.tb3_state == TB3_RIGHT_TURN:
                    if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                        self.tb3_state = GET_TB3_DIRECTION  # 当转向角已经大于阈值的时候，转向足够，返回初始状态重新判断
                    else:
                        self.update_command_velocity(0.0, -1 * ANGULAR_VELOCITY)  # 按照角速度右转，直到转到阈值
                elif self.tb3_state == TB3_LEFT_TURN:  # 同样一直保持左转直到转到阈值
                    if math.fabs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                        self.tb3_state = GET_TB3_DIRECTION
                    else:
                        self.update_command_velocity(0.0, ANGULAR_VELOCITY)
                # 保险用的，如果状态不是上述任意一种，则将状态重置为初始等待状态
                else:
                    self.tb3_state = GET_TB3_DIRECTION
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("异常信息：%s", e)
                rate.sleep()
                continue

# 入口点
if __name__ == '__main__':
    try:
        # 同样，依据tf2.py的脚本，虽然传入了两个参数arg，但会存在一个remapping argument，实际上会接收到3个参数
        if len(rospy.myargv()) != 3:
            rospy.logerr("传入的参数不等于3")  # 若传入的argv不为2,则出现报错消息
            sys.exit(1)  # 退出程序
        # 接收.launch传过来的参数
        target_frame_param = rospy.myargv()[1]
        source_frame_param = rospy.myargv()[2]
        # 创建Drive类的实例
        tb3_drive = Turtlebot3Drive(source_frame_param, target_frame_param)
        tb3_drive.control_loop()
    # 设置一个异常捕捉
    except rospy.ROSInterruptException as e:
        rospy.logerr("错误信息：%s", e)
