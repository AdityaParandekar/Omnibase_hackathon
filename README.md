# Omnibase_hackathon

PROCESS TO RUN THE SIMULATION --

 1) The permission should be given to all the nodes(python files) 
 
         chmod u+x contoller_trial.py
         chmod u+x RRT_path.py
    
 2) After giving permission to the individual nodes, the launch file should be given permission 
     
         chmod u+x ROS_hackathon.launch
         
  
 3) Now, we just need to run the following command to run the simulation
	   
		 roslaunch omnibase_gazebo ROS_hackathon.launch
     
 
 THE NODES USED IN THE WORKING --
 
 1) The CONTROLLER node => This node is a PID controller . It subscribes to the odometry and the path provided by the Path Planning algorithm via a Custom msg (Complex.msg) . It publishes to the topic /cmd_vel


