#!/usr/bin/env python

#Implementation of Pure Pursuit Control Agorithm
#Author: Shivam Sood
#Date: 31/08/2020

import rospy
from scipy import spatial			
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped 
from tf.transformations import euler_from_quaternion
import math

path_list = []
prius_pos = [0,0]
prius_pos_front = [0,0]
car_length_half = 1.5
steer_angle = 0
yaw = 0
distance = 0
eps = 10**(-7)		#For when the velocity of car becomes 0(1/v term gives a divide by zero error)
vel= 0

k=22.5			#Stanley Tuning parameter

def odom_callback(Odometry):
    global prius_pos,val,yaw, car_length_half,prius_pos_front,vel
    prius_pos[0], prius_pos[1] = Odometry.pose.pose.position.x, Odometry.pose.pose.position.y  		#Find the position of the car
    quat =  Odometry.pose.pose.orientation								#Get orientation of the car
    quat_list = [quat.x,quat.y,quat.z,quat.w]
    rall,pitch,yaw = euler_from_quaternion(quat_list) 							#Convert from quaternion to roll, pitch and yaw
    prius_pos_front[0], prius_pos_front[1] = prius_pos[0]+car_length_half*math.cos(yaw), prius_pos[1]+car_length_half*math.sin(yaw)	#Position of front of the car
    vel = math.sqrt(Odometry.twist.twist.linear.x**2 + Odometry.twist.twist.linear.y**2)	
  
def stanley(Path):  										
    global path_list,tan_steer,ld,k,steer_angle,yaw, distance, car_length,prius_pos_front,eps,vel
    path_list = []													#Re-initilize the path everytime the callback is called
    for t in Path.poses:
	path_list.append([t.pose.position.x,t.pose.position.y])
    distance,index = spatial.KDTree(path_list).query(prius_pos_front)							#Returns the distance and the index of the closest point to car in our path
    theta_e = math.atan2((path_list[index+1][1] - path_list[index-1][1]),(path_list[index+1][0]-path_list[index-1][0])) - yaw	#Calculate theta_e for
    if theta_e > math.pi:
        theta_e -= 2*math.pi
    if theta_e < (-1*math.pi):
        theta_e += 2*math.pi													#Make sure it lies btw (-pi to pi)
    check = math.cos(yaw)*(path_list[index][1]-prius_pos_front[1]) - math.sin(yaw)*(path_list[index][0]-prius_pos_front[0])	#To check if the goal point is on the left or right of our car
    sign = 1 
    if check>0:
        sign = 1
    else:
        sign = -1
    steer_angle = (theta_e + math.atan(k*sign*distance/(vel+eps)))*(0.8)	#*(4.0/9)	# (-40 degrees , 40 degrees) #The Stanley formula
    print(sign*distance)
    
def main():
    global prius_pos ,path_list, val,index,steer_angle, distance

    rospy.init_node('pure_pursuit',anonymous=True)
    pub = rospy.Publisher('/prius', Control, queue_size =1000)			#Topic for car's control
    rospy.Subscriber('/base_pose_ground_truth' , Odometry, odom_callback)	#Odometry data
    rospy.Subscriber('/astroid_path' , Path, stanley)				#Path data
    rate=rospy.Rate(30)								#Rate: 30Hz
    move = Control()
    while not rospy.is_shutdown():
        move.steer = steer_angle
        pub.publish(move)
    rate.sleep()

if __name__ == '__main__':
    try:
	    main()
    except rospy.ROSInterruptException:
        pass

