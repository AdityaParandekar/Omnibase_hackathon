#!/usr/bin/env python3
import matplotlib.pyplot as plt
import random
from math import   atan2, pow, sqrt
import rospy
from beginner_tutorials.msg import Complex
import time

startx=0.0
starty=0.0
obstaclex = [0,0,0,1.5,1.5,1.5,1.5,3,3,3,3,4.5,4.5,4.5,4.5]
obstacley = [1.5,3,4.5,0,1.5,3,4.5,0,1.5,3,4.5,0,1.5,3,4.5]
time.sleep(15)
goalx = float(input("Enter goalx = "))
goaly = float(input("Enter goaly = "))
for i in range(len(obstaclex)):
    while not ((sqrt(pow(( obstaclex[i]-goalx), 2) + pow((obstacley[i]-goaly), 2))))>0.7:
        goalx = float(input("Enter a number outside the obstacle(x coordinate) = "))
        goaly = float(input("Enter a number outside the obstacle(y coordinate) = "))
max=0.25
nodes_x = [startx]
nodes_y = [starty]
parent_index= [0]
path=[]
goal_reached= False
                       
while (goal_reached==False):
    min_index = 0
    ranx=random.uniform(0,(goalx+1))
    rany=random.uniform(0,(goaly+1))
    min_distance= sqrt(pow(( nodes_x[0]-ranx), 2) + pow((nodes_y[0]-rany), 2))
    ranx_a=[]
    rany_a=[]
    c1x_a=[]
    c1y_a=[]
    final_pathx_goal_to_start=[]
    final_pathy_goal_to_start=[]
    for i in range(len(nodes_x)):
        distance= sqrt(pow(( nodes_x[i]-ranx), 2) + pow((nodes_y[i]-rany), 2))
        if (distance<min_distance) :
            min_distance = distance
            min_index = i
            theta= atan2((rany-nodes_y[i]),(ranx-nodes_x[i]))      
    if(min_distance<max):
        for i in range(len(obstaclex)):
            if (sqrt(pow(( obstaclex[i]-ranx), 2) + pow((obstacley[i]-rany), 2)))>0.7:
                ranx_a.append(ranx)
                rany_a.append(rany)
        #print(len(ranx_a))        
        if(len(ranx_a)%15==0 and len(rany_a)%15==0):
            nodes_x.append(ranx)
            nodes_y.append(rany) 
            if (sqrt(pow(( ranx-goalx), 2) + pow((rany-goaly), 2))<0.1):
                goal_reached=True
                print('path planned!!!\n\n\n')
            plt.plot([nodes_x[min_index],ranx], [nodes_y[min_index],rany], "r.-", markersize = 3, linewidth = 0.3) 
            parent_index.append(min_index)
                       
    else:
        c1x= (max*(ranx - nodes_x[min_index]))/min_distance+nodes_x[min_index]
        c1y= (max*(rany - nodes_y[min_index]))/min_distance+nodes_y[min_index]
        for i in range(len(obstaclex)):
            if (sqrt(pow(( obstaclex[i]-c1x), 2) + pow((obstacley[i]-c1y), 2)))>0.7:
                c1x_a.append(c1x)
                c1y_a.append(c1y)
        if (len(c1x_a)%15==0 and len(c1y_a)%15==0):
            nodes_x.append(c1x)
            nodes_y.append(c1y)
            if (sqrt(pow(( c1x-goalx), 2) + pow((c1y-goaly), 2))<0.1):
                goal_reached=True    
                print('path planned!!!\n\n\n')                
            plt.plot([nodes_x[min_index],c1x], [nodes_y[min_index],c1y], "r.-", markersize = 3, linewidth = 0.3)
            parent_index.append(min_index)
               
pos = parent_index[len(parent_index)-1]
while ( pos!=0):
    path.append(pos)
    pos = parent_index[pos]

path.append(0)    
  
for i in range(len(path)-1):
   plt.plot([nodes_x[path[i]],nodes_x[path[i+1]]], [nodes_y[path[i]],nodes_y[path[i+1]]], "k.-", markersize = 3, linewidth = 0.3)

for i in range(len(path)):
    final_pathx_goal_to_start.append(nodes_x[path[i]])
    final_pathy_goal_to_start.append(nodes_y[path[i]])
final_pathx = final_pathx_goal_to_start[::-1] 
final_pathx.pop(0)
final_pathy = final_pathy_goal_to_start[::-1]
final_pathy.pop(0)
final_path = []
k=0
while k<len(final_pathx):
    final_path.append([final_pathx[k],final_pathy[k]])
    k=k+1
print(final_path)

rospy.init_node ("Path_planner")
pub_pathx = rospy.Publisher('final_pathx', Complex)
pub_pathy = rospy.Publisher('final_pathy', Complex)

rate = rospy.Rate(10) 
while not rospy.is_shutdown():
    pathx = Complex()
    pathy = Complex()
    pathx.array=final_pathx
    pathy.array=final_pathy
    pub_pathx.publish(pathx)
    pub_pathy.publish(pathy)
    rate.sleep()
    
#plt.show()
