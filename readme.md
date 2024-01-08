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