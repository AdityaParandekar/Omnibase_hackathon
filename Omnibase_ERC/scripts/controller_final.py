#!/usr/bin/env python3
from math import atan, atan2, pi,sqrt
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Twist
from math import atan2, pow, sqrt
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math

x=0.0
y=0.0
phi=0.0
kp_distance=1.0
ki_distance=0.0
kd_distance=0.0

def newod (msg):
    global x
    global y
    global phi
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rotq = msg.pose.pose.orientation
    (roll,pitch,phi)= euler_from_quaternion ([rotq.x,rotq.y,rotq.z,rotq.w])
   
rospy.init_node ("speed_contoller")
sub = rospy.Subscriber("/odom", Odometry , newod )
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

c=0
speed = Twist()
goal = Point()
r = rospy.Rate(4)
total_distance=0.0
prev_distance=0.0
path= [[2,0.8],[2,2.5],[2.5,3.5],[3.7,4],[4,5],[5,6]] 
goal.x= path[0][0]
goal.y= path[0][1]

while not rospy.is_shutdown():       
    distance = sqrt(pow((goal.x - x), 2) + pow((goal.y - y), 2))
    
    if distance< 0.1 and c<=(len(path)-1) :
        goal.x= path[c][0]
        goal.y= path[c][1]
        c=c+1
    theta = atan2((goal.y-y),(goal.x-x))

    diff_distance= distance-prev_distance
   
    u = (
                kp_distance * distance
                + ki_distance * total_distance
                + kd_distance * diff_distance
            )          
    if (0<=theta<=pi):
        speed.linear.x =   min(u *(math.cos(theta)),0.5)
        speed.linear.y = min(u*(math.sin(theta)),0.5)
    else:
        speed.linear.x =   min(u *(-1)*(math.cos(theta)),0.5)
        speed.linear.y = min(u*(-1)*(math.sin(theta)),0.5)
    pub.publish(speed)
    prev_distance = distance
    total_distance = total_distance + distance
    
    r.sleep()