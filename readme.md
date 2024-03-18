### 写了人能看的中文

#### -2024/2/25/-

##### Linux系统操作：

1. 刚进terminal的时候，如果有未找到文件类似的杂项，调用主用户权限，sudo vi ~/.bashrc，进入了vim编辑，按下a启动编辑模式，在最底层删掉不必要的source等然后按ESC最后:wq保存     
2. 启动Pycharm进行编辑：cd Downloads/pycharm-community-2023.3.3/bin ->  ./pycharm.sh，使用chmod a+rwx xx.py修改.py文件的权限让其可以被作为程序脚本运行，在PyCharm中Setting中安装CMake高亮提示书写.launch文件，其中对.py文件需要在最上面加上shebang行，告诉操作系统用哪个解释器来解释脚本#!/usr/bin/env python3
3. 快捷键打开终端：CTRL+ALT+T，对终端的指令：mkdir xx 创建文件夹，touch xx.py 创建一个文件，ls 查看当前路径下的所有文件
4. 运行ROS，先编辑其工作空间workspace，src为代码空间，build为编译空间，devel为开发空间，跑出roscore的指令：cd ~/catkin_ws && catkin_make -> source ./devel/setup.bash -> roscore      
5. 基础的跟随程序，是由服务器到客户端的编程模式实现的，先创建ROSMaster，分别对Client和Server实现注册功能，然后由Client(turtle_spawn)发送请求，途中经过Service(/spawn)最终抵达Server(turtle_sim)，再由Server端发送response到Service最后被Client接收     
6. 中文输入法的切换用CTRL+SPACE键    
7. 实现Windows/PC与虚拟机之间的通信(移动文件夹，复制粘贴内容等)需要安装VMwaretool，需要在VMware中把CD的权限放开，之后解压安装包运行install文件即可     
8. 使用rosrun rqt_graph rqt_graph检查当前topic的图             

#### -2024/2/26/-

##### 双龟通信实现：基于roscore

1. 对catkin_ws中的src文件夹安装需要的软件包，catkin_create_pkg turtlefollowroscpp rospy std_msgstf2 tf2_geometry_msgs tf2_ros turtlesim，其次，在src中创造client.py，publisher.py，subscriber.py，再额外在src同级目录中创建launch文件夹，其中放置turtle_track.launch       
2. 环境变量配置好使用roslaunch roscore turtle_track.launch      

#### -2024/2/27/-

##### 下载对应noetic版本的TurtleBot3-devel功能包，实现自主SLAM建图和路径规划

1. Gazebo插件用来构建给机器人导航的仿真和建模环境，而除开官方示例定制自己Gazebo世界，首先在命令台sudo gazebo ->找到editor中的build editor -> 滚轮调整比例尺大概4m左右为真实环境 -> 使用默认包/网上的材质包搭建 -> save world as..保存在world路径下面      
2. 修改turtlebot3中范例的内容，将roslaunch中默认的world改为model.world，初始x,y,z改为0，0，0，之后配置环境变量后再指定模型：source ./devel/setup.bash -> export TURTLEBOT3_MODEL=burger，打开世界：roslaunch turtlebot3_gazebo turtlebot3_test.launch
3. 开SLAM建图指令，此处采用算法gmapping，roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping，键盘控制指令：roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch，建好图后保存：rosrun map_server map_saver -f /home/offmind/catkin_ws/src/turtlebot3-noetic-devel，运行自主导航命令，roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/offmind/catkin_ws/src/turtlebot3-noetic-devel.yaml       
4. 当所有配置好(使用gazebo11版本，常见命令，gazebo --verbo, killall gzserver)，打开顺序：cd catkin_ws -> source ./devel/setup.bash -> export TURTLEBOT3_MODEL=burger -> roslaunch turtlebot3_gazebo turtlebot3_test.launch -> roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/offmind/catkin_ws/src/turtlebot3-noetic-devel.yaml              

#### -2024/2/29/-

##### 实现在Gazebo世界中创建多辆小车：

1. 在src中创建分别的三个文件夹：multi-robot-gazebo, multi-robot-nav, multi-robot-test
2. 先处理multi-robot-gazebo里面的，按照turtlebot3_simulation里面的gazebo package安装gazebo里面必要的安装包：catkin_create_pkg roscpp rospy sensor_msgs geometry_msgs nav_msgs tf gazebo_ros urdf xacro gazebo_plugins，
3. 在multi-robot-gazebo环境中会出现三个文件：src，CMakeLists.txt, package.xml，再在同一个文件夹中创建world和launch文件夹，其中world使用别人创建好的box_house.world，最后即在launch文件中创建三个.launch文件：multi_robot_formation.launch，one_robot.launch，turtlebot3_world.launch，运行的时候：roslaunch multi-robot-gazebo multi_robot_formation.launch，就可以在gazebo世界上看见三辆小车         
4. 对ROS编译空间的概念：参考0229里面的ROSFile，里面会有CMakeLists.txt和package.xml，如果使用了catkin_create_pkg指令，从最上面看，最顶层是catkin工作空间，是ROS工程中层次最高的概念，而src是源空间，用于创建功能包，CMake文件中规定了功能包的编译规则，包括指定功能包等名称，指定要生成或者要添加的库文件等，而package中定义了功能包的属性信息，包括包名，版本号，编译依赖和运行依赖，剩下的include是存放c++的头文件.h的地方，则在src中存放对应的源程序文件.cpp，因为整体采用.py书写，意味着并不需要c++的头文件以及include文件夹，而是将脚本存放在scripts的地方。         
5. ROS社区最为常用的SLAM算法为gmapping算法，它根据移动机器人里程计数据和激光雷达来绘制二维的栅格化地图，而SLAM算法是属于导航之前，用于构建全局地图的，当机器人读取到gmapping算法SLAM建好的二维地图，在开始导航的时候，机器人需要判断当前位置，而AMCL，蒙特卡洛定位算法，是根据已有的地图使用粒子滤波器推算机器人的位置。到最后，move_base是获得了激光雷达，二维地图map和蒙特卡洛(amcl)的定位数据，再根据给定的目标点，实现了机器人导航。      
6. 有关具体one_robot.launch功能包里面的两个joint_state_publisher和robot_state_publisher从属关系放在图片0301里，joint_state_publisher通过订阅小车上/robot_description节点获得机器人上面非固定关节的信息,name,position,velocity,effort等，将这些信息通过/joint_state发布到/move_group与/robot_state_publisher节点上，而/robot_state_publisher节点接收到来自/joint_state的信息，在.launch文件下面指定发送给/move_group的tf坐标并指定发送频率，详情见csdn博客：https://blog.csdn.net/weixin_36965307/article/details/104945052      
7. multi-robot-gazebo文件相互关系说明:<img src="https://github.com/0FFMIND/Turtle-Bot/blob/master/20240301_MultiGazebo.png">

#### -2024/3/12/-

##### 小车纵队的实现：

1. 导航的关键技术，SLAM(Simultaneous Localization and Mapping)，实时定位与并发式的地图构建，而问题可被描述为将一个机器人放入未知环境中的未知位置，使其开始移动，并在移动的过程中在自身定位的基础上建造出增量式地图，以绘制出外部环境的完全地图(边探索边绘制并补全完整地图)，在ROS中保存地图的功能包是map_server。其中，若机器人要完成SLAM，机器人必须具备感知外界环境的能力，尤其是具备获取周围环境深度信息的能力，其依赖于传感器，在Turtlebot3_burger中通过最顶上的激光雷达实现           
2. 坐标系：现在使用的坐标关系包为TF2，以下用TF代替，在双龟跟随系统中，出现了三个坐标系：world frame(世界坐标系)，turtle1 frame(乌龟1的坐标系)，turtle2 frame(跟随龟乌龟2的坐标系)，程序中使用TF，分别广播两个乌龟的互相坐标系，再在乌龟2里创建listener，监听tf坐标并在脚本里面计算位置差，从而使得乌龟2跟随乌龟1移动      
3. 坐标系的创建与发送：
   - 两种常见的载体，TransformStamped和PoseStamped，均在geometry_msg.msg的空间下，TransformStamped用来传输坐标系相关的位置信息，它首先存在header指明frame_id，父坐标系，其次是child_frame_id，子坐标系，分别为两个字段，接下来transform中储存了一个用Vector3表示的translation,xyz坐标，还存在一个用四元数Quaternion表示的rotation,xyzw坐标
   - 在实际使用上，TransformStamped，使用broadcaster = TransformBroadcaster()创造一个，再使用tfs = TransformStamped()创建一个空信息等待发送，在最后调用broadcaster.sendTransform(tfs)即使用广播器广播信息，并依次设置父坐标系，子坐标系，时间(time_stamp)，transform等
   - 最后在主入口__main__中发布一个node，dynamic_tf_pub，动态变换，再创建sub = rospy.Subscriber("xx/odom",Odometry,callback=包括广播器的函数)，第一个参数表示其要根据xx/odom里程计的数据，传入的数据类型为Odometry，通过回调函数来设置tf坐标系的数据，而PoseStamped常配合listener使用，提前创造缓冲区，获取Publisher发布的数据，详情见知乎：https://zhuanlan.zhihu.com/p/680520864       
4. 坐标系定位：
   - 一种是基于里程计定位(odom)，时时收集机器人的速度信息，计算并发布机器人坐标系与父级参考系的相对关系，其收集的信息是连续的，但缺点是里程计存在累计误差，并不利于长距离或者长期定位
   - 一种是基于传感器定位(map)，它通过传感器收集外界环境信息，通过匹配计算发布机器人坐标系与父级参考系的相对关系，它比里程计更精准但是传感器定位会出现跳变的情况，且在周围标志物较少的环境下，定位精度会大打折扣(根据自己周围环境判断位置)，详情见Gitcode博客：https://gitcode.csdn.net/65e842d61a836825ed78c03a.html        
5. 对multi-robot-test里面的功能包安装：catkin_create_pkg roscpp rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros dynamic_reconfigure        
6. 建立scripts文件夹 -> tf2_broadcaster.py 和 turtlebot3_slave.py书写.py脚本       
7. 建立launch文件夹，同样创建三个launch文件：multi_robot_formation.launch，multi_robot_tf2.launch，turtlebot3_slave.launch进行书写，启动文件为multi_robot_formation.launch      

#### -2024/3/17/-

##### 小车自动寻路，全局代价和局部代价的实现：

1. multi-robot-nav里面的功能包安装：catkin_create_pkg roscpp rospy actionlib amcl geometry_msgs gmapping map_server move_base move_base_msgs       
2. 整体参考官方给出的navigation包move_base.launch的书写，新建launch文件夹，创建三个文件：multi_mapping.launch，algorithm_gmapping.launch，teb_local_planner.launch书写，启动文件为multi_mapping.launch      
3. 使用SLAM中的Gmapping算法进行平面移动机器人定位和建图 ( Hector和Cartographer的设计初衷用于地面不平坦的情况 ) 
   - Gmapping是在RBpf的基础上改进，RBpf使用粒子滤波来估计机器人的位姿，而粒子滤波采用的是重要性重采样算法，包括四个步骤：采样-计算权重-重采样-地图估计，通过不断的迭代来估计每一时刻机器人的位姿
   - 由于激光雷达会返回360度每一度的位置信息，其中包括距离和角度，对360个点进行位置估计计算量过大，这时引入了改进提议分布，从运动模型中采集粒子，由于每一个粒子都带有一个地图，使用观测对粒子加权以选出表现最好的粒子
   - 而改进提议分布围绕最近的一次观测来模拟目标分布(目标分布即根据机器人所携带的传感器数据确定机器人的状态置信度最大极限，结合里程计和激光雷达数据形成机器人的位姿估计，但由于传感器带有噪声，故机器人位姿存在不确定度)，这时候如果由于环境相似度高或者测量噪声的影响，导致正确的粒子被分配的较小权重被丢弃，此时需要进行重采样最后确定位姿(但是频繁的重采样又会导致相同权重小的粒子被丢弃导致频繁重采样)        
4. 对move_base.launch的自定写法，首先move_base功能包主要实现机器人导航功能，在上述的SLAM中已经构建好灰度地图了(实际上为2D图片，有宽度，高度，标度尺等元数据，并使用灰度值表示障碍物存在的概率)，但这个地图并不能直接被导航使用，因为SLAM构建的地图为静态地图       
5. 常见的寻路算法：Dijkstra迪杰斯特拉算法和A\*算法，Dijkstra
