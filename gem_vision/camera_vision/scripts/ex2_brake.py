#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/13/2021                                                          
# Version: 0.1                                                                    
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
from camera_vision.msg import Detected_msg
import message_filters

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

import sys
souce_path = "./src/gem_vision/camera_vision/scripts/" # for simulator
# souce_path = "./src/vehicle_drivers/gem_vision/gem_vision/camera_vision/scripts/" # for real car

# sys.path can be used to add additional directories to the Python module search path, allowing ROS nodes to import modules or packages from custom locations.
# To be specify, if you want to import a function from some module you write, you need to add its path by sys.path.append()
sys.path.append(souce_path)

class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(10)
        self.prev_x     = 0.0

        self.look_ahead = 4
        self.wheelbase  = 1.75 # meters
        self.offset     = 0.46 # meters

        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 1

        self.olat       = 40.0928563
        self.olon       = -88.2359994

        # read waypoints into the system 
        self.goal       = 0

        self.desired_speed = 0.55  # m/s, reference speed
        self.max_accel     = 0.36 # % of acceleration
        self.pid_speed     = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)
        

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = True

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = True

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear  = False
        self.brake_cmd.ignore = False

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear  = False
        self.accel_cmd.ignore = False

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        self.objects   = rospy.Subscriber("/object_detection", Detected_msg, self.object_callback)
        self.middle_x = 0.0
        self.middle_z = 1.0

    def object_callback(self, objects):
        # print("object_detection:", objects)
        self.middle_z = objects.middle_y

    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees

    def speed_callback(self, msg):
        self.speed = round(msg.data, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        if (heading_curr >= 0 and heading_curr < 90):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 90 and heading_curr < 180):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 180 and heading_curr < 270):
            yaw_curr = np.radians(90 - heading_curr)
        else:
            yaw_curr = np.radians(450 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):

        if(f_angle > 35):
            f_angle = 35

        if (f_angle < -35):
            f_angle = -35

        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)

        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0

        return steer_angle

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

    def start_pp_ex2(self):
        # if (self.gem_enable == False):

        #         if(self.pacmod_enable == True):

        #             # ---------- enable PACMod ----------

        #             # enable forward gear
        #             self.gear_cmd.ui16_cmd = 3

        #             # enable brake
        #             self.brake_cmd.enable  = True
        #             self.brake_cmd.clear   = False
        #             self.brake_cmd.ignore  = False
        #             self.brake_cmd.f64_cmd = 0.0

        #             # enable gas 
        #             self.accel_cmd.enable  = True
        #             self.accel_cmd.clear   = False
        #             self.accel_cmd.ignore  = False
        #             self.accel_cmd.f64_cmd = 0.0

        #             self.gear_pub.publish(self.gear_cmd)
        #             # print("Foward Engaged!")

        #             self.turn_pub.publish(self.turn_cmd)
        #             # print("Turn Signal Ready!")
                    
        #             self.brake_pub.publish(self.brake_cmd)
        #             # print("Brake Engaged!")

        #             self.accel_pub.publish(self.accel_cmd)
        #             # print("Gas Engaged!")

        self.gear_cmd.ui16_cmd = 3
        # self.gear_pub.publish(self.gear_cmd)

        while not rospy.is_shutdown():
            
            print("start_pp!")

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    # self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0
                  
                    # self.gear_pub.publish(self.gear_cmd)
                    # print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    # print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    # print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    # print("Gas Engaged!")

            
            curr_x, curr_y, curr_yaw = self.get_gem_state()
            # print("curr_x",curr_x)
            sign = curr_x - self.prev_x
            self.prev_x = curr_x
            # print("Current X Y Yaw",curr_x, curr_y, curr_yaw)

            print("middle_z: ",  self.middle_z)
    

            # if self.middle_z > 10:
            #     # self.accel_cmd.f64_cmd = output_accel
            #     # self.steer_cmd.angular_position = np.radians(steering_angle)
            #     # self.brake_cmd.f64_cmd = 0.0
            #     self.accel_cmd.f64_cmd = 0.35
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     # self.brake_pub.publish(self.brake_cmd)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     # self.turn_pub.publish(self.turn_cmd)
            #     # print("acceleration: ",self.accel_cmd.f64_cmd)
            #     print("acceleraton!")
            #     self.rate.sleep()

            # else:
            #     print("break!")
            #     self.brake_cmd.f64_cmd = 1.0
            #     # self.accel_cmd.f64_cmd = 0.0
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.brake_pub.publish(self.brake_cmd)
            #     # self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     # self.turn_pub.publish(self.turn_cmd)
            #     # print("brake: ",self.brake_cmd.f64_cmd)
            #     self.rate.sleep()
                        ## compute expected velocity

            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)

            ## distance
            distance = 6.0
            v_expect = self.middle_z - distance

            if v_expect > 0:
                if self.gear_cmd.ui16_cmd == 3:
                    output_accel = 0.32
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)
                    self.rate.sleep()
                else:
                    output_brake = 0.42
                    self.brake_cmd.f64_cmd = output_brake
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.brake_pub.publish(self.brake_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)
                    self.rate.sleep()
                    if abs(filt_vel) < 0.005:
                        self.gear_cmd.ui16_cmd = 3
                

            else:
                if self.gear_cmd.ui16_cmd == 3:
                    output_brake = 0.42
                    self.brake_cmd.f64_cmd = output_brake
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.brake_pub.publish(self.brake_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)
                    self.rate.sleep()
                    if abs(filt_vel) < 0.005:
                        self.gear_cmd.ui16_cmd == 1
                else:
                    output_accel = 0.32  
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)
                    self.rate.sleep()
                    
            # self.gear_cmd.ui16_cmd = 1
            # self.gear_pub.publish(self.gear_cmd)
            # print("-------------------------------")
            # print("self.gear_cmd", self.gear_cmd.ui16_cmd)
            

        
            
            # if sign < 0:
            #     filt_vel = -abs(filt_vel)
            # if sign >= 0:
            #     filt_vel = abs(filt_vel)
            # a_expect = self.pid_speed.get_control(current_time, v_expect - filt_vel)
            # print("filt_vel", filt_vel)
            # print("sign", sign)
##-------------------------------------------------------------------------------
            # a_expect = v_expect - filt_vel
            # if a_expect >= 0.32:
            #     a_expect = 0.32

            # if a_expect > 0.31:
            #     # self.gear_cmd.ui16_cmd = 3                    
            #     # self.gear_pub.publish(self.gear_cmd)
            #     # print("Foward Engaged!")
            #     output_brake = 0.0
            #     output_accel = (a_expect + 2.3501) / 7.3454   
            #     # self.accel_cma_expect = v_expect - filt_vel
            # if a_expect >= 0.32:
            #     a_expect = 0.32

            # if a_expect > 0.31:
            #     # self.gear_cmd.ui16_cmd = 3                    
            #     # self.gear_pub.publish(self.gear_cmd)
            #     # print("Foward Engaged!")
            #     output_brake = 0.0
            #     output_accel = (a_expect + 2.3501) / 7.3454   
            #     # self.accel_cmd.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()
            # elif a_expect <= 0.31 and a_expect >= 0:
            #     # self.gear_cmd.ui16_cmd = 3                    
            #     # self.gear_pub.publish(self.gear_cmd)
            #     # print("Foward Engaged!")
            #     output_accel = 0.31
            #     output_brake = d.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()
            # elif a_expect <= 0.31 and a_expect >= 0:
            #     # self.gear_cmd.ui16_cmd = 3                    
            #     # self.gear_pub.publish(self.gear_cmd)
            #     # print("Foward Engaged!")
            #     output_accel = 0.31
            #     output_brake = 0.0  
            #     # self.accel_cmd.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()

            # elif a_expect < 0 and a_expect >= -0.4:
            #     # self.gear_cmd.ui16_cmd = 1                  
            #     # self.gear_pub.publish(self.gear_cmd)
            #     # print("Foward Engaged!")
            #     output_brake = 0.0
            #     output_accel = (a_expect + 2.3501) / 7.3454   
            #     # self.accel_cmd.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()

            # else:
            #     # self.gear_cmd.ui16_cmd = 1                   
            #     # self.gear_pub.publish(self.gear_cmd)
                
            #     # print("Foward Engaged!")
            #     output_accel = 0.31
            #     output_brake = 0.0  
            #     # self.accel_cmd.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()
    ##------------------------------------------------------------------------
            print("filt_vel ", filt_vel)

            # elif a_expect < 0 and a_expect >= -0.4:
            #     output_brake = -0.31
            #     output_accel = 0.0
            #     # self.brake_cmd.f64_cmd = output_brake # new for brake
            #     self.brake_cmd.f64_cmd = abs(output_brake)
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.brake_pub.publish(self.brake_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)

            #     self.rate.sleep()

            # else:
            #     output_brake = 0.3758*a_expect -0.01842
            #     output_accel = 0.0
            #     # self.brake_cmd.f64_cmd = output_brake # new for brake
            #     self.brake_cmd.f64_cmd = abs(output_brake)
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.brake_pub.publish(self.brake_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)

            #     self.rate.sleep()
            
            # if self.middle_z > 9:
            #     # self.accel_cmd.f64_cmd = output_accel
            #     self.accel_cmd.f64_cmd = output_accel
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

            #     self.rate.sleep()

            # else:                                     # new for brake
            #     output_accel = 0.0
            #     # self.brake_cmd.f64_cmd = output_brake # new for brake
            #     self.brake_cmd.f64_cmd = abs(output_brake)
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.brake_pub.publish(self.brake_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)

            #     self.rate.sleep()



            # if self.middle_z > 7:
            #     # self.accel_cmd.f64_cmd = output_accel
            #     # self.steer_cmd.angular_position = np.radians(steering_angle)
            #     self.accel_cmd.f64_cmd = 0.35
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.accel_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)
            #     print("acceleraton!")
            #     self.rate.sleep()

            # else:
            #     print("break!")
            #     # self.brake_cmd.f64_cmd = -1.0
            #     self.brake_cmd.f64_cmd = 1.0
            #     self.steer_cmd.angular_position = np.radians(0.0)
            #     self.brake_pub.publish(self.accel_cmd)
            #     self.steer_pub.publish(self.steer_cmd)
            #     self.turn_pub.publish(self.turn_cmd)
            #     print("self.accel_cmd.f64_cmd",self.brake_cmd.f64_cmd)

            
            #     self.rate.sleep()


def start_pp_ex3(self):

        self.gear_cmd.ui16_cmd = 3

        while not rospy.is_shutdown():
            
            print("start_pp!")

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    # self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0
                  
                    # self.gear_pub.publish(self.gear_cmd)
                    # print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    # print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    # print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    # print("Gas Engaged!")

            # get current velocity and direction
            curr_x, curr_y, curr_yaw = self.get_gem_state()
            # print("curr_x",curr_x)
            sign = curr_x - self.prev_x
            self.prev_x = curr_x
            # print("Current X Y Yaw",curr_x, curr_y, curr_yaw)
    
            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)
            
            print("middle_z: ",  self.middle_z)
            ## distance
            distance = 6.0
            v_expect = self.middle_z - distance

            a_expect = self.pid_speed.get_control(current_time, v_expect - filt_vel)



            if v_expect > 0:
                if self.gear_cmd.ui16_cmd == 3:
                    output_accel = 0.32
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)
                    self.rate.sleep()
                else:
                    output_brake = 0.42
                    self.brake_cmd.f64_cmd = output_brake
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.brake_pub.publish(self.brake_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)
                    self.rate.sleep()
                    if abs(filt_vel) < 0.005:
                        self.gear_cmd.ui16_cmd = 3
                

            else:
                if self.gear_cmd.ui16_cmd == 3:
                    output_brake = 0.42
                    self.brake_cmd.f64_cmd = output_brake
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.brake_pub.publish(self.brake_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)
                    self.rate.sleep()
                    if abs(filt_vel) < 0.005:
                        self.gear_cmd.ui16_cmd == 1
                else:
                    output_accel = 0.32  
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)
                    self.rate.sleep()
                    
            


def pure_pursuit():

    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp_ex3()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


