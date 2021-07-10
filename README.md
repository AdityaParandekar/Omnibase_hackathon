# Omnibase_hackathon
SETUP--
Install dependencies(from omnibase repository: https://github.com/ERC-BPGC/omnibase) --

          sudo apt install python-catkin-tools
          sudo apt install ros-noetic-joint-state-controller
          sudo apt install ros-noetic-effort-controllers
          sudo apt install ros-noetic-joint-trajectory-controller
          sudo apt install ros-noetic-position-controllers
          sudo apt install ros-noetic-velocity-controllers
          
Package installation from debian --

             sudo apt update
             sudo apt install ros-noetic-omnibase-control 
             sudo apt install ros-noetic-omnibase-gazebo 
             sudo apt install ros-noetic-omnibase-description


PROCESS TO RUN THE SIMULATION --
 1) Create a workspace

          cd ~/catkin_ws/src
          git clone https://github.com/AdityaParandekar/Omnibase_hackathon.git
          cd ..
          catkin build
          source devel/setup.bash

 1) The permission should be given to all the nodes(python files) 

         cd catkin_ws/src/Omnibase_ERC/scripts/
         chmod u+x controller.py
         chmod u+x RRT_Path.py
    
 2) After giving permission to the individual nodes, the launch file should be given permission 
 
     
         cd catkin_ws/src/omnibase/omnibase_gazebo/launch/
         chmod u+x ROS_ERC_Hackathon.launch
         
  
 3) Now, we just need to run the following command to run the simulation
	   
		 roslaunch omnibase_gazebo ROS_ERC_Hackathon.launch
     
 
 THE NODES USED IN THE WORKING --
 
 1) The CONTROLLER node => This node is a PID controller . It subscribes to the odometry and the path provided by the Path Planning algorithm via a Custom msg (Complex.msg) . It publishes to the topic /cmd_vel . The omnibase moves along the path generated by the path planning node . 

 2) The PATH PLANNER node => This node uses RRT algorithm to find the path from starting point to the goal . It publishes the path and sends it as a custom msg to which the PID controller subscribes . The RRT algorithm generates random points (nodes) and these nodes are connected in a way so that the entire environment is explored . To consider the obstacles in our environment , a condition is put on the nodes that if the point generated is at a distance suitable distance from the center of all the obstacles i.e outside the circle (the obstacle) then it is consider for connection (forming an edge) . The nodes are added until the tree reaches the goal . A path is obtained from this tree and this path is published . 


The path planner expores the map (avoiding obstacles) :


![image](https://user-images.githubusercontent.com/84431866/125171349-68f17080-e1d1-11eb-8092-45b367c59ac5.png)


After the user enters the final goal points ( x and y coordinates) a path is formed which is taken by the omnibot --

![image](https://user-images.githubusercontent.com/84431866/125171626-d2be4a00-e1d2-11eb-8db3-b5fd458a5a41.png)


And finally goal is reached 

![image](https://user-images.githubusercontent.com/84431866/125171686-0ac58d00-e1d3-11eb-84c0-e92c440cc58a.png)





