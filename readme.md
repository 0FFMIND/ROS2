# Process Recorded     
***
-2024/1/06-     
Tasks: Initialize Project Settings   
VMware: username-offmind password-0*****              
Ubuntu: 20.04LTS ROS:Noetic         
Install package: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/      
***
-2024/1/08-    
Tasks: Start to build QUIC server using golang        
Server: Tencent Cloud with Ubuntu 22.04 LTS      
Framework: https://github.com/traefik/traefik document: https://doc.traefik.io/traefik/      
First, we need to install docker to our system, https://linux.cn/article-14871-1.html,      
and see the command: sudo docker run hello-world runs successfully.       
Next is to install Docker Compose and follow the same tutorial above.     
--> sudo -s --> cd compose --> docker-compose up -d reverse-proxy --> open http://localhost:8080/api/rawdata
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240108_DockerCompose.png">
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240108_ReverseProxy.webp"> 
***    
-2024/1/09-    
Tasks: ROS Simulation      
Build workspace: mkdir catkin_ws --> cd catkin_ws --> mkdir src --> cd src --> git... -->          
cd ~/catkin_ws && catkin_make --> source ./devel/setup.bash         
To avoid each catkin_make and we need to add environmental variable, we use sudo gedit ~/.bashrc to change the file. source ~/catkin_ws/.... -> export ROS_PACKAGE...         
In ROS Gazebo, we obtain same turtlebot controlled by key value
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240109_ROSGazebo.jpg">          
***
-2024/1/10-        
Tasks: Simple ROS Topic       
RPC(Remote Procedure Call) --> Call methods between different services as if they were local methods between the same service       
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240110_RPCDetails.jpg">          
Topic Communication(Async)：Ros Node(Publisher) --> Topic:/example + Message Type: std_msgs/String --> multiple Ros Node(Subsriber)       
Client/Server Communication(Sync):ROS Node(Service Client) -(request)-> Ros Node(Service Server) -(response)-> Ros Node(Service Client)        
Commands Listed:      
roscore --> Start ROS Master      
rosrun turtlesim turtlesim_node --> Start simulation      
rosrun turtlesim turtle_teleop_key --> Start key control      
rosrun rqt_graph rqt_graph --> Show system graph       
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240110_ROSTopic.png">     
***           
-2024/1/11-        
Tasks: WorkSpace customized     
Structure of workspace:    
src --> Source Space      
build --> Build Space     
devel --> Development Space    
install --> Install Space     
command: catkin_create_pkg <package_name> [depend1][depend2][depend3]..., Then we cd to src, and catkin_create_pkg learning_topic geometry_msgs turtlesim std_msgs rospy roscpp, then complie them use catkin_make, and add the environmental variable: source ~/catkin_ws/devel/setup.bash
Then create .py --> touch velocity_publisher.py --> add #!/usr/bin/env python3 at the first line
***        
-2024/1/12-     
Tasks: Solve the problem of unexpected Trojan attack to our server      
Detective --> Remove --> reset SSH and VNC password     
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240112_Trojan.png"> 
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240112_Remove.png"> 
***           
-2024/1/16-      
Tasks: MAS (Multi Agent System) in Unity for Simulation
Unity3D version: 2020.3.20f1c1, Unity Package: Behavior Designer - Behavior Trees for Everyone; Behavior Designer - Movement Pack        
The blue balls are designed to move randomly using Unity AI, and the red balls have two actions: if the blue ball collide with them, the count will plus one, and if two red balls are too closer, it will move far away.     
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240116_AgenntB.png">  
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240116_MAS.png"> 
***            
-2024/1/17/-      
Tasks: Basics of Docker and Traefik      
traefik is an edge router, when we start traefik, we need to first define entrypoints,  and based on the routers connected to these entrypoints, we need to see if these routers match a set of rules. If the rules match, then the routers we input will go through middlewares. After the conversion, it is forwarded to the real services.      
For traefik, to discover the services, we need to use connectors which are called providers because they provide the configuration to Traefik. Provider can automatically discover services on the platform, and the provider could be Docker or Kubernetes, etc.. Here we use Docker Provider for traefik configuration, and for Docker startup, we use Docker Compose here. For compose, it simplifies the control of the entire application stack, which can easily manage services in a single easy-to-understand YAML configuration file. Then, using a single command, you can create and start all the services from the profile. the simple example in https://docs.docker.com/compose/gettingstarted/ to run Docker Compose         
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240117_traefik.png"> 
***         
-2024/1/18-       
Tasks: Learn Simple Sample of Docker Compose    
1. mkdir composetest -> cd composetest -> touch app.py / touch requirement.txt -> add flask,redis     
2. create Dockerfile, in same dir, touch Dockerfile, and the Dockerfile is used to create Docker Image.       
3. touch compose.yaml -> docker compose up -> open http://localhost:8000/
Error Message: Error response from daemon: Head "https://registry-1.docker.io/v2/library/redis/manifests/alpha": net/http: TLS handshake timeout    
Solution: https://blog.csdn.net/qq_35606010/article/details/104750391 create deamon.json in etc/docker  -> sudo nano etc/docker/deamon.json and Solution for From ...alpine: https://www.cnblogs.com/xiaoyao404/p/14266360.html add RUN set -eux && sed -i 's/dl-cdn.alpinelinux.org/mirrors.ustc.edu.cn/g' /etc/apk/repositories in Dockerfile      
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240118_DockerCompose.png">
***            
-2024/1/21-       
Tasks: Simple implementation of NAT traversal     
1. mkdir nat -> touch docker-compose.yaml -> sudo docker compose up       
2. install cpolar in Linux -> create account -> sudo systemctl enable cpolar -> sudo systemctl start cpolar -> localhost:9200        
3. successful! -> https://1bc0b503.r3.cpolar.cn/dashboard/       
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240121_DockerCommand.png">        
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240121_nattraversal.png">
***            
-2024/1/22-     
Tasks: TLS challenges for Traefik       
1. The concern is to use https on a simple service exposed with Traefik, and TLS stands for Transport Layer Secure. TLS is often used to combine with HTTP -> https -> more secured       
2. Shake hands mechanism for TLS -> Client sends 'Client Hello' message to Server -> Server sends 'Server Hello' message to Client, including Certificate, SercerKeyExchange, CertificateRequest, ServerHelloDone -> Client processes 'Server Hello' and sends 'ClientKeyExchange' --> Server 'Finished?' --> Exchange Data       
3. While we use Docker to run traefik, here are three files needed: docker-compose.yaml & dynamic.yaml & traefik.yaml(Optional)         
4. mkdir tls -> cd tls -> touch docker-compose.yaml -> version: "3.3" service: traefik: -> ports: 80(http)443(https)8080(dashboard) tbc.....         
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240122_StaticYaml.png"> 
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240122_TraefikOperation.png"> 
***            
-2024/1/23/-       
Tasks: Implement ROS Publisher and Subscriber     
1. touch publisher_velocity.py -> init ROS node -> create rospy.Publisher('name',Twist,queue_size=10)       
2. touch pose_subscriber.py -> init ROS node -> create rospy.Subscriber('name',Pose,poseCallback) for register callback func         
3. run cd /home/offmind -> cd catkin_ws -> catkin_make -> source ~/catkin_ws/devel/setup.bash -> roscore ->  rosrun turtlesim turtlesim_node -> rosrun learning_topic velocity_publisher.py -> same pose_subscriber.py        
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240123_Publisher&Subscriber.png"> 
<img src="https://github.com/0FFMIND/TurtleBot/blob/master/20240123_Simulation.png"> 