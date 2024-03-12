# 写了人能看的中文版     

***            

-2024/2/25/-       

熟悉Linux系统：      
1.刚进terminal的时候，如果有未找到文件类似的杂项，直接调用主用户权限，sudo vi ~/.bashrc，此时进入了vim编辑，按下a启动编辑模式，往下到最底层，删掉不必要的source等然后按ESC最后:wq保存     
2.启动Pycharm进行编辑：cd Downloads -> cd pycharm-community-2023.3.3 -> cd bin -> ./pycharm.sh，而修改xx.py的权限使用chmod a+rwx xx.py让其可以被访问和修改，最后.launch文件是CMake编写，在PyCharm中Setting中安装CMake高亮提示，其中对.py文件需要在最上面加上shebang行，告诉操作系统用哪个解释器来解释脚本#!/usr/bin/env python3     
3.快捷键打开终端：CTRL+ALT+T，对终端的指令：mkdir xx 创建文件夹，touch xx.py 创建一个文件，ls 查看当前路径下的所有文件     
4.运行ROS，先编辑其的工作空间，src为代码空间，build为编译空间，devel为开发空间，跑出roscore的指令：cd ~/catkin_ws && catkin_make -> source ./devel/setup.bash -> roscore      
5.基础的跟随程序，是由服务器到客户端的编程模式实现的，先创建ROS      Master，分别对Client和Server实现注册功能，然后由Client(turtle_spawn)发送请求，途中经过Service(/spawn)最终抵达Server(turtle_sim)，再由Server端发送response到Service最后被Client接收     
6.中文输入法的切换用CTRL+SPACE键    
7.实现Windows/PC与虚拟机之间的通信(移动文件夹，复制粘贴内容等)需要安装VMwaretool，需要在VMware中把CD的权限放开，之后解压安装包运行install文件即可     

***            

-2024/2/26/-       
双龟通信实现：基于roscore      
1.对catkin_ws中的src文件夹安装需要的软件包，catkin_create_pkg turtlefollowroscpp rospy std_msgstf2 tf2_geometry_msgs tf2_ros turtlesim，其次，在src中创造client.py，publisher.py，subscriber.py，再额外在src同级目录中创建launch文件夹，其中放置turtle_track.launch       
2.环境变量配置好使用roslaunch roscore turtle_track.launch，使用rosrun rqt_graph rqt_graph检查当前topic的图      

***            

-2024/2/27/-       
1.实现自主SLAM建图和路径规划：需要下载对应noetic版本的TurtleBot3-devel功能包       
2.ROS中的Gazebo插件用来构建给机器人导航的仿真和建模环境，而除开官方示例定制自己Gazebo世界，首先在命令台sudo gazebo ->找到editor中的build editor -> 滚轮调整比例尺大概4m左右为真实环境 -> 使用默认包/网上的材质包搭建 -> save world as..保存在world路径下面      
3.修改turtlebot3中范例的内容，将roslaunch中默认的world改为model.world，初始x,y,z改为0，0，0，之后配置环境变量后再指定模型：source ./devel/setup.bash -> export TURTLEBOT3_MODEL=burger，打开世界：roslaunch turtlebot3_gazebo turtlebot3_test.launch，之后再开一个SLAM建图指令，此处采用算法gmapping，roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping，键盘控制指令：roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch，建好图后保存：rosrun map_server map_saver -f /home/offmind/catkin_ws/src/turtlebot3-noetic-devel，最后运行自主导航命令，roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/offmind/catkin_ws/src/turtlebot3-noetic-devel.yaml       
4.当所有配置好(使用gazebo11版本，常见命令，gazebo --verbo, killall gzserver)，打开顺序：cd catkin_ws -> source ./devel/setup.bash -> export TURTLEBOT3_MODEL=burger -> roslaunch turtlebot3_gazebo turtlebot3_test.launch，之后roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/offmind/catkin_ws/src/turtlebot3-noetic-devel.yaml              

***            

-2024/2/29/-       
1.实现多车跟随系统：在单车SLAM与路径规划中进行多车领队跟随       
2.在src中创建分别的三个文件夹：multi-robot-gazebo, multi-robot-nav, multi-robot-test, 先处理multi-robot-gazebo里面的，首先安装gazebo里面的功能包，按照turtlebot3_simulation里面的gazebo, package必要的安装包：catkin_create_pkg roscpp rospy sensor_msgs geometry_msgs nav_msgs tf gazebo_ros urdf xacro gazebo_plugins，之后在multi-robot-gazebo环境中会出现三个文件：src，CMakeLists.txt, package.xml，再在同一个文件夹中创建world和launch文件夹，其中world使用别人创建好的box_house.world，最后即在launch文件中创建三个.launch文件：multi_robot_formation.launch，one_robot.launch，turtlebot3_world.launch，运行的时候：roslaunch multi-robot-gazebo multi_robot_formation.launch，就可以在gazebo世界上看见三辆小车         
3.下面介绍一些对ROS编译空间的概念：参考0229里面的ROSFile，里面会有CMakeLists.txt和package.xml如果使用了catkin_create_pkg指令，从最上面看，最顶层是catkin工作空间，是ROS工程中层次最高的概念，而src是源空间，用于创建功能包，CMake文件中规定了功能包的编译规则，包括指定功能包等名称，指定要生成或者要添加的库文件等，而package中定义了功能包的属性信息，包括包名，版本号，编译依赖和运行依赖，剩下的include是存放c++的头文件.h的地方，则在src中存放对应的源程序文件.cpp，我们使用.py书写，意味着并不需要c++的头文件，而是存放在scripts的地方。         
4.ROS社区最为常用的SLAM算法为gmapping算法，它根据移动机器人里程计数据和激光雷达来绘制二维的栅格化地图，而SLAM算法是属于导航之前，用于构建全局地图的，当机器人读取到gmapping算法SLAM建好的二维地图，在开始导航的时候，机器人需要判断当前位置，而AMCL，蒙特卡洛定位算法，是根据已有的地图使用粒子滤波器推算机器人的位置。到最后，move_base是获得了激光雷达，二维地图map和蒙特卡洛(amcl)的定位数据，再根据给定的目标点，实现了机器人导航。      
5.有关具体one_robot.launch功能包里面的两个joint_state_publisher和robot_state_publisher从属关系放在图片0301里，详情见csdn博客：https://blog.csdn.net/weixin_36965307/article/details/104945052      
6.multi-robot-gazebo文件相互关系说明:<img src="https://github.com/0FFMIND/Turtle-Bot/blob/master/20240301_MultiGazebo.png">

***            

-2024/3/12/-       
1.实现导航的关键技术：首先考虑SLAM(Simultaneous Localization and Mapping)，实时定位与并发式的地图构建，而问题可被描述为将一个机器人放入未知环境中的未知位置，使其开始移动，并在移动的过程中在自身定位的基础上建造出增量式地图，以绘制出外部环境的完全地图(边探索边绘制并补全完整地图)，在ROS中保存地图的功能包是map_server。其中，若机器人要完成SLAM，机器人必须具备感知外界环境的能力，尤其是具备获取周围环境深度信息的能力，其依赖于传感器，在Turtlebot3_burger中通过最顶上的激光雷达实现
详情见Gitcode博客：https://gitcode.csdn.net/65e842d61a836825ed78c03a.html
1.对multi-robot-test里面的功能包安装：catkin_create_pkg roscpp rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros dynamic_reconfigure
2.对用C++写的头文件.h，建立scripts文件夹->里面对头文件turtlebot3_drive.h进行python重写，
3.对launch文件夹，同样创建三个launch文件：multi_robot_formation.launch，multi_robot_tf2.launch，turtlebot3_slave.launch



2.最后multi-robot-nav里面的功能包安装：catkin_create_pkg roscpp rospy actionlib amcl geometry_msgs gmapping map_server move_base move_base_msgs       

