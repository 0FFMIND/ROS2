# 写了人能看的中文版     

-2024/2/25/-       

熟悉Linux系统：      
1.刚进terminal的时候，如果有未找到文件类似的杂项，直接调用主用户权限，sudo vi ~/.bashrc，此时进入了vim编辑，按下a启动编辑模式，往下到最底层，删掉不必要的source等然后按ESC最后:wq保存     
2.启动Pycharm进行编辑：cd Downloads -> cd pycharm-community-2023.3.3 -> cd bin -> ./pycharm.sh，而修改xx.py的权限使用chmod a+rwx xx.py让其可以被访问和修改     
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
2.       