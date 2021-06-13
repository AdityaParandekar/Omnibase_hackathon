#!/usr/bin/env python3
from math import atan, atan2, pi,sqrt

from numpy.core.numeric import array_equal
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import arcball_nearest_axis, euler_from_quaternion
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Twist
from math import atan2, pow, sqrt
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
from beginner_tutorials.msg import Complex
x=0.0
y=0.0
phi=0.0
kp_distance=1.0
ki_distance=0.0
kd_distance=0.0
global path
global arr_x
global arr_y
arr_y = []
arr_x = []
path = [[0,0]]

def x_sub (msg):
    global path
    global arr_x
    #print(msg.array)
    arr_x=msg.array
    

def y_sub (msg):
    global path
    global arr_y
    arr_y= msg.array
    k=0

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
sub_pathx = rospy.Subscriber("final_pathx",Complex,x_sub)
sub_pathy = rospy.Subscriber("final_pathy",Complex,y_sub)
print(sub_pathx)

c=0
speed = Twist()
goal = Point()
r = rospy.Rate(4)
total_distance=0.0
prev_distance=0.0
#path= [[2,0.8],[2,2.5],[2.5,3.5],[3.7,4],[4,5],[5,6]] 
#path = [[0.22978594390358031, 0.09848055637708712], [0.4126154386171657, 0.26898972288368594], [0.6569599144279317, 0.3218648317796683], [0.6708733784107963, 0.571477362554863], [0.874725437139985, 0.7161989509647179], [1.1013204569022121, 0.8218147500619927], [1.3476197538363541, 0.7789585214627874], [1.5975918603036317, 0.7826929542593534], [1.8280588524644472, 0.8795689859924222], [2.0280403909951024, 1.0295935981296047], [2.176367980227017, 1.2308370483898514], [2.2412389855558548, 1.4722738986584587], [2.273458026592035, 1.7201890724472144], [2.226112759415917, 1.965664989139712], [2.249360812500938, 2.2145816981565156], [2.1421153689845966, 2.4404099843497717], [2.1510468587091487, 2.6902503903919093], [2.217870972322963, 2.9311539765829995], [2.2393046555136205, 3.18023347639156], [2.1305620836084795, 3.4053446779241696], [2.217267067468005, 3.6398276107229544], [2.064214495199048, 3.8375013573911847], [2.0804993912277556, 4.0869703978778436], [2.232020830655178, 4.285820224113227], [2.2080021751720156, 4.534663757661481], [2.2566154809887777, 4.779891705696704], [2.35705613774572, 5.008827667201737], [2.596608050516004, 5.080346064052465], [2.8166852797726927, 5.198946288178303], [2.9653511946750486, 5.399939934212812], [3.130888221311587, 5.587283182856402], [3.2937662762599156, 5.776942717855962], [3.1989253691474486, 6.008254633541437], [3.3622612283739914, 6.197520047948632], [3.5701032308706204, 6.336450613333615], [3.807550517708994, 6.414673282510108], [3.972285616899647, 6.602722071643794], [4.203989360328694, 6.508842493028601], [4.430628068654385, 6.403320476897073], [4.6244180294959625, 6.561261766865908], [4.859333130126976, 6.646788929191806], [5.105985557610274, 6.687563676482212], [5.313351329300256, 6.5479232928841355], [5.530795955562894, 6.6712838802266985], [5.76659632244834, 6.5882285554323845], [6.015531664290029, 6.611276236056715], [6.238687455368507, 6.723977136266691], [6.428951850262801, 6.886148218878567], [6.583620347282398, 7.0825601737838095], [6.833607383812073, 7.085106065987101]]

goal.x= path[0][0] 
goal.y= path[0][1] 


while not rospy.is_shutdown():
    k=0
    while k< (len(arr_x)):
        path.append([arr_x[k],arr_y[k]])
        k=k+1
       
    distance = sqrt(pow((goal.x - x), 2) + pow((goal.y - y), 2))
    
    if distance< 0.4 and c<=(len(path)-1) : 
        goal.x= path[c][0]
        goal.y= path[c][1]
        c=c+1
    
    theta = atan2((goal.y-y),(goal.x-x))

    diff_distance= distance-prev_distance
    u = (kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance)
   

    speed.linear.x =   min(u *(math.cos(theta)),0.5)
    speed.linear.y = min(u*(math.sin(theta)),0.5)

    pub.publish(speed)
    prev_distance = distance
    total_distance = total_distance + distance
    
    r.sleep()
    
