#!/usr/bin/env python

#Implementation of Pure Pursuit Control Agorithm
#Author: Shivam Sood
#Date: 31/08/2020

import rospy
from scipy import spatial			#to find the nearest point
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped 
from tf.transformations import euler_from_quaternion	#for finding yaw from a quaternion
import math

path_list = []
prius_pos, prius_pos_front = [0,0], [0,0]
car_length = 3
index = 0
steer_angle = 0
ld=0
yaw = 0
distance = 0
vel = 0

kl=10		# how many indexes ahead to compute the look ahead distance
kt = 0.08        # the tuning parameters

def pos_callback(Odometry):
    global prius_pos,val,yaw,prius_pos_front,vel
    prius_pos[0], prius_pos[1] = Odometry.pose.pose.position.x, Odometry.pose.pose.position.y  		#Find the position of the car
    quat =  Odometry.pose.pose.orientation								#Quaternion Orientation of the car
    quat_list = [quat.x,quat.y,quat.z,quat.w]			
    roll,pitch,yaw = euler_from_quaternion(quat_list) 							#Function to find roll,pitch,yaw from quaternions
    prius_pos_front[0], prius_pos_front[1] = prius_pos[0]+car_length*math.cos(yaw)/2.0, prius_pos[1]+car_length*math.sin(yaw)/2.0	#Calculate the co-ordinates of the middle of the front axle
    vel = math.sqrt(Odometry.twist.twist.linear.x**2 + Odometry.twist.twist.linear.y**2)
  
def pure_pursuit(Path):  										
    global path_list, index,tan_steer,ld,k,steer_angle,yaw, distance, car_length, gamma
    path_list = []								# Re-initialize it to empty for every iteration of callback
    for t in Path.poses:
	path_list.append([t.pose.position.x,t.pose.position.y])			# take the path data and append the positions into an array

    distance,index = spatial.KDTree(path_list).query(prius_pos_front)		# finds the distance and index of the nearest point in the path array w.r.t prius_pos_front
    ld = [path_list[index+kl][0] , path_list[index+kl][1]]			# the look ahead distance (a certain indexes ahead from the nearest point)

    dot_product = math.cos(yaw)*(ld[0]-prius_pos_front[0])+ math.sin(yaw)*(ld[1]-prius_pos_front[1])	#dot-product between ld vector and car's heading unit vector
    abs_prod = math.sqrt((ld[0]-prius_pos_front[0])**2+(ld[1]-prius_pos_front[1])**2)			#absolute value to find alpha (the anngle between the vectors)
    alpha = abs(math.acos(dot_product/abs_prod))							#Find alpha(absolute value as we will assign sign later on)
    tan_steer = 2*car_length*math.sin(alpha)/(kt*vel)							#The pure pursuit formula
    check = math.cos(yaw)*(path_list[index+kl][1]-prius_pos_front[1]) - math.sin(yaw)*(path_list[index+kl][0]-prius_pos_front[0])	#Determine whether the look-ahead point is on left or right 
																	#or right of the car (left :+ve, right:-ve)
    sign =1 
    if check>0:
        sign = 1
    else:
        sign = -1								#assign sign according to check
    steer_angle = sign*(math.atan(tan_steer))*(4.0/9)				#Find the steering angle and map it according to max and min steering angles physicaly possible(-40 deg to +40 deg here)
    print(sign*distance)
    
def main():
    global prius_pos ,path_list, val,index,steer_angle, distance

    rospy.init_node('pure_pursuit',anonymous=True)
    pub = rospy.Publisher('/prius', Control, queue_size =1000)			#Car's control topic
    rospy.Subscriber('/base_pose_ground_truth' , Odometry, pos_callback)	#Car's Odometry data
    rospy.Subscriber('/astroid_path' , Path, pure_pursuit)			#The topic on which path is being published
    rate=rospy.Rate(30)
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

