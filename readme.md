# Process Recorded     
***
-2024/1/06-     
Tasks: Initialize Project Settings   
VMware: username-offmind password-0*****              
Ubuntu: 20.04LTS ROS:Noetic         
Install package: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/      
***
-2023/1/08-    
Tasks: Start to build QUIC server using golang        
Server: Tencent Cloud with Ubuntu 22.04 LTS      
Framework: https://github.com/traefik/traefik document: https://doc.traefik.io/traefik/      
First, we need to install docker to our system, https://linux.cn/article-14871-1.html,      
and see the command: sudo docker run hello-world runs successfully.       
Next is to install Docker Compose and follow the same tutorial above.     
--> sudo -s --> cd compose --> docker-compose up -d reverse-proxy --> open http://localhost:8080/api/rawdata
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240108_DockerCompose.png">      
***    
-2023/1/09-    
Tasks: ROS Simulation
Build workspace: mkdir catkin_ws --> cd catkin_ws --> mkdir src --> cd src --> git... -->          
cd ~/catkin_ws && catkin_make --> source ./devel/setup.bash         
To avoid each catkin_make and we need to add environmental variable, we use sudo gedit ~/.bashrc to change the file. source ~/catkin_ws/.... -> export ROS_PACKAGE...         
In ROS Gazebo, we obtain same turtlebot controlled by key value
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240109_ROSGazebo.jpg">          