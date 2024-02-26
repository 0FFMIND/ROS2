# 写了人能看的中文版     

-2024/2/24/-       

1.刚进terminal的时候，如果有未找到文件类似的杂项，直接调用主用户权限，sudo vi ~/.bashrc，此时进入了vim编辑，按下a启动编辑模式，往下到最底层，删掉不必要的source等然后按ESC最后:wq保存     
2.启动Pycharm进行编辑：cd Downloads -> cd pycharm-community-2023.3.3 -> cd bin -> ./pycharm.sh，而修改xx.py的权限使用chmod a+rwx xx.py让其可以被访问和修改     
3.快捷键打开终端：CTRL+ALT+T，对终端的指令：mkdir xx 创建文件夹，touch xx.py 创建一个文件，ls 查看当前路径下的所有文件     
4.运行ROS，先编辑其的工作空间，src为代码空间，build为编译空间，devel为开发空间，跑出roscore的指令：cd ~/catkin_ws && catkin_make -> source ~/catkin_ws/devel/setup.bash -> roscore      
5.基础的跟随程序，是由服务器到客户端的编程模式实现的，先创建ROS      Master，分别对Client和Server实现注册功能，然后由Client(turtle_spawn)发送请求，途中经过Service(/spawn)最终抵达Server(turtle_sim)，再由Server端发送response到Service最后被Client接收     
6.中文输入法的切换用CTRL+SPACE键    
7.实现Windows/PC与虚拟机之间的通信(移动文件夹，复制粘贴内容等)需要安装VMwaretool，需要在VMware中把CD的权限放开，之后解压安装包运行install文件即可

***            

-2024/2/25/-       