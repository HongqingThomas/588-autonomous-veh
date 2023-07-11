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
        self.read_waypoints() 

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

    # def read_waypoints(self):

    #     # read recorded GPS lat, lon, heading
    #     dirname  = os.path.dirname(__file__)
    #     filename = os.path.join(dirname, '../waypoints/xy_demo.csv')

    #     with open(filename) as f:
    #         path_points = [tuple(line) for line in csv.reader(f)]

    #     # print(path_points)

    #     # x towards East and y towards North
    #     self.path_points_lon_x   = [float(point[0]) for point in path_points] # longitude
    #     self.path_points_lat_y   = [float(point[1]) for point in path_points] # latitude
    #     self.path_points_heading = [float(point[2]) for point in path_points] # heading
    #     self.wp_size             = len(self.path_points_lon_x)
    #     self.dist_arr            = np.zeros(self.wp_size)

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

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def start_pp(self):
        
        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

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

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    # self.gem_enable = True


            # self.path_points_x = np.array(self.path_points_lon_x)
            # self.path_points_y = np.array(self.path_points_lat_y)

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            print("curr_x",curr_x)
            # print("Current X Y Yaw",curr_x, curr_y, curr_yaw)

            print("middle_z: ",  self.middle_z)
            print("type", type(self.middle_z))
            # if self.middle_z < 1000:
            #     middle_dist = self.middle_z
            # else:
            #     middle_dist = 1.0

            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)
            print("filt_vel ", filt_vel)

## comments begins ----------------------------------------------------------------
            # ## --------- continue ----------##
            # if(self.gem_enable == True):
            #     print("Current index: " + str(self.goal))
            #     print("Forward velocity: " + str(self.speed))
            #     ct_error = round(np.sin(alpha) * L, 3)
            #     print("Crosstrack Error: " + str(ct_error))
            #     print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
            #     print("Steering wheel angle: " + str(steering_angle) + " degrees" )
            #     print("\n")



            # ## compute expected velocity
            # v_expect = self.middle_z - 3.0

            # # method1
            # # current_time = rospy.get_time()
            # # filt_vel     = self.speed_filter.get_data(self.speed)
            # # output_brake = self.pid_speed.get_control(current_time, v_expect - filt_vel)

            # # method2
            # a_expect = v_expect - filt_vel
            # output_brake = f(a_expect)



            # current_time = rospy.get_time()
            # filt_vel     = self.speed_filter.get_data(self.speed)
            # output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

            # if output_accel > self.max_accel:
            #     output_accel = self.max_accel

            # if output_accel < 0.3:
            #     output_accel = 0.3

            # if (f_delta_deg <= 30 and f_delta_deg >= -30):
            #     self.turn_cmd.ui16_cmd = 1
            # elif(f_delta_deg > 30):
            #     self.turn_cmd.ui16_cmd = 2 # turn left
            # else:
            #     self.turn_cmd.ui16_cmd = 0 # turn right
## comments ends ----------------------------------------------------------------

## ------------------------------- self.middle_z <= 7.0 ----------------------------------------##

            if self.middle_z > 7.0:
            ## test condition:
            # if curr_x > 7:
                if self.gear_cmd.ui16_cmd == 3:     ## forward mode
                    print("-------------------------------")
                    print("self.gear_cmd", self.gear_cmd.ui16_cmd)

                    output_brake = 0.0              ## acclerate
                    output_accel = 0.33
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

                    self.rate.sleep()

                else:                               ## reverse mode   
                    print("-------------------------------")     
                    print("self.gear_cmd", self.gear_cmd.ui16_cmd)

                    if filt_vel > 0.005:           ## brake
                        output_accel = 0.0
                        output_brake = 0.44
                        self.brake_cmd.f64_cmd = output_brake
                        self.steer_cmd.angular_position = np.radians(0.0)
                        self.brake_pub.publish(self.brake_cmd)
                        self.steer_pub.publish(self.steer_cmd)
                        self.turn_pub.publish(self.turn_cmd)
                        print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)

                        self.rate.sleep()

                    else:
                        self.gear_cmd.ui16_cmd = 3  ## change to forward mode 
                        self.gear_pub.publish(self.gear_cmd)
                        
                        self.rate.sleep()           ## shoule we send an accel cmd here?

## ------------------------------- self.middle_z <= 7.0 ----------------------------------------##

            if self.middle_z <= 7.0:
                if self.gear_cmd.ui16_cmd == 1:     ## reverse mode
                    print("-------------------------------")
                    print("self.gear_cmd", self.gear_cmd.ui16_cmd)

                    output_brake = 0.0              ## acclerate
                    output_accel = 0.33
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(0.0)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    print("self.accel_cmd.f64_cmd",self.accel_cmd.f64_cmd)

                    self.rate.sleep()


                else:                               ## forward mode   
                    print("-------------------------------")     
                    print("self.gear_cmd", self.gear_cmd.ui16_cmd)

                    if filt_vel > 0.005:           ## brake
                        output_accel = 0.0
                        output_brake = 0.44
                        self.brake_cmd.f64_cmd = output_brake
                        self.steer_cmd.angular_position = np.radians(0.0)
                        self.brake_pub.publish(self.brake_cmd)
                        self.steer_pub.publish(self.steer_cmd)
                        self.turn_pub.publish(self.turn_cmd)
                        print("self.brake_cmd.f64_cmd",self.brake_cmd.f64_cmd)

                        self.rate.sleep()

                    else:
                        self.gear_cmd.ui16_cmd = 1  ## change to forward mode 
                        self.gear_pub.publish(self.gear_cmd)

                        self.rate.sleep()           ## shoule we send an accel cmd here?

def pure_pursuit():

    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()

