#!/usr/bin/env python3

#detects the edge of the container

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import plotly.graph_objects as go
import plotly.express as px
import math

n=1


def callback(msg):
    global n
    r_lidar = []
    theta_lidar_deg = []
    theta_lidar_rad = []

    angle_deg = 0
    angle_rad = 0

    left_points = []
    left_point_angle = []

    right_points = []
    right_point_angle = []

    nearest_left_point = 100
    nearest_left_point_angle = 0

    nearest_right_point = 100
    nearest_right_point_angle = 0

    x_points = []
    y_points = []

    lidar_angle_range = (len(msg.ranges))
    angle_rad_incrt = msg.angle_increment
    angle_deg_incrt = math.degrees(msg.angle_increment)

    for i in msg.ranges:
        
        #polar plot
        r_lidar.append(i)
        theta_lidar_deg.append (angle_deg)

        
        if  angle_deg > 90 and angle_deg < 135: 
            left_points.append(i)
            left_point_angle.append(angle_deg)

            if nearest_left_point > i:
                nearest_left_point = i
                nearest_left_point_angle = angle_deg
                print (nearest_left_point)
        
        if  angle_deg < 90 and angle_deg > -135: 
            right_points.append(i)
            right_point_angle.append(angle_deg)

            if nearest_right_point > i:
                nearest_right_point = i
                nearest_right_point_angle = angle_deg

        angle_deg = angle_deg + angle_deg_incrt
        angle_rad = angle_rad + angle_rad_incrt
        


    #draw graph once
    while (n>0):
        print (nearest_right_point)
        print (nearest_left_point)

        fig = go.Figure()

        fig.add_trace(go.Scatterpolar(r=r_lidar,theta=theta_lidar_deg,fill='toself',name='Lidat'))
        fig.add_trace(go.Scatterpolar(r=left_points,theta=left_point_angle,fill='toself',name='Left'))
        fig.add_trace(go.Scatterpolar(r=right_points,theta=right_point_angle,fill='toself',name='Right'))
        fig.add_trace(go.Scatterpolar(r=[nearest_left_point],theta=[nearest_left_point_angle],fill='toself',name='Left closest point'))
        fig.add_trace(go.Scatterpolar(r=[nearest_right_point],theta=[nearest_right_point_angle],fill='toself',name='Right closest point'))

        # fig.update_layout(polar=dict(radialaxis=dict(visible=True,range=[0, 5])),showlegend=False)
        fig.update_layout(polar=dict(radialaxis=dict(visible=True), ),showlegend=False)

        fig.show()

        n = n-1

#   #If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
#   if msg.ranges[lidar_angle_range] > 1:
#       move.linear.x = 0.5
#       move.angular.z = 0.0

#   #If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
#   if msg.ranges[360] < 1:
#       move.linear.x = 0.0
#       move.angular.z = 0.0

#   pub.publish(move)

rospy.init_node('container_detector')
sub = rospy.Subscriber('/mirabb_rev2/laser/front_scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()

rospy.spin()