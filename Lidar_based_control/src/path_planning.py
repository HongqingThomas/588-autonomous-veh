#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from sensor_msgs.msg import Imu
import csv
from copy import deepcopy
import math

class path_planning(object):
    
    def __init__(self):
        # self.a = 1
        self.rate       = rospy.Rate(10)

        # self.look_ahead = 4
        # self.wheelbase  = 1.75 # meters
        self.offset     = 0.46 # meters

        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.olat       = 40.0928563
        self.olon       = -88.2359994

        # read waypoints into the system 
        # self.goal       = 0            
        self.read_waypoints()


        # self.cube = np.array([[0,0],[10,10]])

        # self.read_waypoints() 

    def read_waypoints(self):
        # Define the planning cube
        self.A = [[3,30],[-11,-1]]

        # Define the cylinder obstacles
        # [[25.231465245116457, -4.923283873156118], [28.331879857381683, -10.412998011077805]]
        self.c1_x, self.c1_y, self.r1 = 25.231465245116457, -4.923283873156118, 3
        self.c2_x, self.c2_y, self.r2 = 28.331879857381683, -10.412998011077805, 1
        
        
        # Define the step size for the path planning algorithm
        self.step_size = 0.1

        # Define the maximum turning angle for the car
        self.max_turning_angle = 30

    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees
        # self.east_v  = inspva_msg.east_velocity
        # self.north_v = inspva_msg.north_velocity
        # print("++++++++++++++***********")
        # print(f'{self.lat}, {self.lon}, {self.heading}')
        # print(f'{self.east_v}, {self.north_v}')
        self.start_pp()

    def heading_to_yaw(self, heading_curr):
        # # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        # if (heading_curr >= 0 and heading_curr < 90):
        #     yaw_curr = np.radians(90 - heading_curr)
        # elif(heading_curr >= 90 and heading_curr < 180):
        #     yaw_curr = np.radians(90 - heading_curr)
        # elif(heading_curr >= 180 and heading_curr < 270):
        #     yaw_curr = np.radians(90 - heading_curr)
        # else:
        #     yaw_curr = np.radians(450 - heading_curr)
        # return yaw_curr

        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   

    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # Define the function to check if a point is inside an obstacle
    def is_inside_obstacle(self, x, y):
        if ((x - self.c1_x) ** 2 + (y - self.c1_y) ** 2) <= self.r1 ** 2:
            return True
        elif ((x - self.c2_x) ** 2 + (y - self.c2_y) ** 2) <= self.r2 ** 2:
            return True
        else:
            return False

    def generate_path1(self, start, goal, step_size, max_turning_angle):
        # Initialize the path with the start point
        path = [start]
        # print("path: ", path)

        # Initialize the current point as the start point
        current_point = start
        iter_count = 0
        # Loop until the current point reaches the goal point
        # while current_point != goal:
        while ((current_point[0] - goal[0])**2 + (current_point[1] - goal[1])**2) > 0.5**2:
            if iter_count > 300:
                print(">1000")
                path_list = [list(t) for t in path]
                return path_list
            iter_count += 1
            # print("in loop", iter_count)
            # Calculate the distance and angle to the goal point
            dx = goal[0] - current_point[0]
            dy = goal[1] - current_point[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            angle = math.atan2(dy, dx)

            # Calculate the maximum turning angle based on the distance to the goal point
            max_turning_angle = int(min(max_turning_angle, math.degrees(math.atan2(2 * step_size, distance))))

            # Initialize the best point and best cost
            best_point = None
            best_cost = float('inf')

            # Loop through all possible next points
            for i in range(-max_turning_angle, max_turning_angle + 1, 5):
                # Calculate the next point and cost
                next_point = (current_point[0] + step_size * math.cos(angle + i), current_point[1] + step_size * math.sin(angle + i), angle + i)
                cost = distance + math.sqrt((next_point[0] - goal[0]) ** 2 + (next_point[1] - goal[1]) ** 2)

                # Check if the next point is inside an obstacle
                if self.is_inside_obstacle(next_point[0], next_point[1]):
                    continue

                # Check if the cost is better than the current best cost
                if cost < best_cost:
                    best_point = next_point
                    best_cost = cost

            # Add the best point to the path
            path.append(best_point)
            # print("path: ", path)

            # # Update the current point
            current_point = best_point
            # current_point[0] = best_point[0]
            # current_point[1] = best_point[1]
            # current_point[2] = best_point[2]
        path_list = [list(t) for t in path]
        # Return the path
        return path_list
    
    def generate_path(self, start, goal, step_size, max_turning_angle):
        # Initialize the path with the start point
        path = [start]
        # print("path: ", path)

        # Initialize the current point as the start point
        current_point = start
        iter_count = 0
        # Loop until the current point reaches the goal point
        # while current_point != goal:
        while ((current_point[0] - goal[0])**2 + (current_point[1] - goal[1])**2) > 0.5**2:
            if iter_count > 1000:
                path_list = [list(t) for t in path]
                return path_list
            iter_count += 1
            # print("in loop", iter_count)
            # Calculate the distance and angle to the goal point
            dx = goal[0] - current_point[0]
            dy = goal[1] - current_point[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            angle = math.atan2(dy, dx)

            # Calculate the maximum turning angle based on the distance to the goal point
            # max_turning_angle = int(min(max_turning_angle, math.degrees(math.atan2(2 * step_size, distance))))

            # Initialize the best point and best cost
            best_point = None
            best_cost = float('inf')

            # Loop through all possible next points
            for i in range(-max_turning_angle, max_turning_angle + 1, 5):
                # Calculate the next point and cost
                # print("i:", i, np.radians(i))
                # print("angle:", angle)
                next_point = (current_point[0] + step_size * math.cos(angle + np.radians(i)), current_point[1] + step_size * math.sin(angle + np.radians(i)), angle + np.radians(i))
                cost = distance + math.sqrt((next_point[0] - goal[0]) ** 2 + (next_point[1] - goal[1]) ** 2)

                # Check if the next point is inside an obstacle
                if self.is_inside_obstacle(next_point[0], next_point[1]):
                    continue

                # Check if the cost is better than the current best cost
                if cost < best_cost:
                    best_point = next_point
                    best_cost = cost

            # Add the best point to the path
            path.append(best_point)
            # print("path: ", path)

            # # Update the current point
            current_point = best_point
            # current_point[0] = best_point[0]
            # current_point[1] = best_point[1]
            # current_point[2] = best_point[2]
        path_list = [list(t) for t in path]
        # Return the path
        return path_list

    def path1_smoothing(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
        new_path = deepcopy(path)
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(len(path[0])):
                    aux = new_path[i][j]
                    new_path[i][j] += weight_data * (path[i][j] - new_path[i][j])
                    new_path[i][j] += weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j] - (2.0 * new_path[i][j]))
                    change += abs(aux - new_path[i][j])
        return new_path

    def generate_path2(self, start_yaw, smooth_path1):
        angles = np.linspace(0, 2 * np.pi, 1000)
        for angle in angles:
            if angle + start_yaw < np.pi:
                point_yaw = angle+start_yaw
            else:
                point_yaw = -np.pi * 2 + (angle+start_yaw)
            # point_x = self.c1_x + self.r1 * np.cos(point_yaw)
            # point_y = self.c1_y + self.r1 * np.sin(point_yaw)
            point_x = self.c1_x + self.r1 * np.sin(point_yaw)
            point_y = self.c1_y - self.r1 * np.cos(point_yaw)
            smooth_path1.append([point_x, point_y, point_yaw])         
        return smooth_path1  

    def start_pp(self):
        
        # while not rospy.is_shutdown():

        # curr_x, curr_y, curr_yaw = self.get_gem_state()
        # Generate the path
        # self.start = (curr_x, curr_y, curr_yaw)
        self.start =(20.011, -7.693, 0.0402)
        goal_x, goal_y =(self.c1_x + self.c2_x)/2, (self.c1_y + self.c2_y)/2
        yaw_goal = np.arctan((self.c1_x - self.c2_x) / (self.c2_y - self.c1_y))
        self.goal = (goal_x, goal_y, yaw_goal)
        # path1 = self.generate_path1(self.start, self.goal, self.step_size, self.max_turning_angle) # after heading_to_yaw()
        path1 = self.generate_path(self.start, self.goal, self.step_size, self.max_turning_angle) # after heading_to_yaw()
        # print("path:", path)
        # Smooth the path
        smooth_path1 = self.path1_smoothing(path1, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001)
        # print("smooth_path:", smooth_path)
        smooth_path = self.generate_path2(yaw_goal, smooth_path1)

        self.rate.sleep()
        print("success!")
        with open("/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control_/waypoints/smoothpath.csv", "w", newline='') as csvfile:
            # writer.truncate()
            writer = csv.writer(csvfile)
            writer.writerows(smooth_path)

def path_planning_function():

    rospy.init_node('pathplanning', anonymous=True)
    pp = path_planning()
    rospy.spin()
    pp.start_pp()
    # try:
    #     pp.start_pp()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    path_planning_function()


# recording