#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from Lidar_based_control.msg import box
from sensor_msgs.msg import Image,PointCloud2
# import ros_numpy
import numpy as np
import ros_numpy
from gazebo_msgs.msg import ModelStates
import csv

from cv_bridge import CvBridge, CvBridgeError
import message_filters

# import alvinxy.alvinxy as axy # Import AlvinXY transformation module
# from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

class Lidar_based_detector():

    def __init__(self):
        self.search_range = 10
        self.ground = -0.3
        self.search_area = [[-10,10],[-10,20]]

        self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        # self.sub_point_cloud = rospy.Subscriber('/lidar1/velodyne_points',PointCloud2, self.pointcloud_callback,queue_size=1)
        self.sub_point_cloud = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback,queue_size=1)    # 仿真的的点云数据
        # self.sub_point_cloud = message_filters.Subscriber('/lidar1/velodyne_points',PointCloud2)    # gem上面的点云数据

    def pointcloud_callback(self, lidar_data):
        # print("lidar data:", lidar_data)
        # # Convert PointCloud2 data to numpy
        # pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(lidar_data.astype(np.float64))#.astype(np.float64)
        frame_id = lidar_data.header.frame_id
        # rospy.loginfo("received point cloud with frame_id: %s", frame_id)
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_data)
        # pc = np.asarray(pc.tolist()).astype(np.float32)[:,:4]
        # pc = np.asarray(pc.tolist()).astype(np.float32)[:,:3]

        pc = np.asarray(pc.tolist())[:,:3]
        pc_near = pc[(pc[:,0]**2 + pc[:,1]**2)< self.search_range**2]
        
        self.state = np.array([self.x, self.y, self.z])
        pc_near += self.state
        
        pc_high = pc_near[pc_near[:,2] > self.ground]

        pc_inrange = pc_high[pc_high[:,0] > self.search_area[0][0]]
        pc_inrange = pc_inrange[pc_inrange[:,0] < self.search_area[0][1]]
        pc_inrange = pc_inrange[pc_inrange[:,1] > self.search_area[1][0]]
        pc_inrange = pc_inrange[pc_inrange[:,1] < self.search_area[1][1]]
        

        
        print("gem_state:", self.state)
        print("shape:", pc_inrange.shape)
        print("relative averagex:", np.mean(pc_inrange[:,0]))

        
        # pc = np.asarray(pc.tolist())[:,:4]
        # pc[:,3] = 0
        # print("shape:", pc.shape)
        # print("averagex:", np.mean(pc[:,0]))
        with open("./src/Lidar_based_control/src/detection.csv", "a") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(pc_inrange[:, :2])
    
    def state_callback(self, state_data):
        model_index = state_data.name.index("gem")
        position = state_data.pose[model_index].position
        self.x = position.x
        self.y = position.y
        self.z = position.z
        # rospy.loginfo("position: x = %f, y = %f, z = %f", position.x, position.y, position.z)


class Lidar_based_detector_1():

    def __init__(self):
        self.search_range = 10
        self.ground = -0.3
        self.search_area = [[-10,10],[-10,20]]

        # self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        # # self.sub_point_cloud = rospy.Subscriber('/lidar1/velodyne_points',PointCloud2, self.pointcloud_callback,queue_size=1)
        # self.sub_point_cloud = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback,queue_size=1)    # 仿真的的点云数据
        # # self.sub_point_cloud = message_filters.Subscriber('/lidar1/velodyne_points',PointCloud2)    # gem上面的点云数据

        self.sub_state = message_filters.Subscriber('/gazebo/model_states', ModelStates)
        self.sub_point_cloud = message_filters.Subscriber('/velodyne_points', PointCloud2)
        sync = message_filters.ApproximateTimeSynchronizer([self.sub_state, self.sub_point_cloud], 1, 0.1, allow_headerless=True)
        # print("self.subcriber_rgb, self.depth_img_sub, self.subcriber_rgb_camera", self.subcriber_rgb, self.depth_img_sub, self.subcriber_rgb_camera)
        sync.registerCallback(self.perception_multi_callback)

    def perception_multi_callback(self, state_data, lidar_data):
        model_index = state_data.name.index("gem")
        position = state_data.pose[model_index].position
        self.x = position.x
        self.y = position.y
        self.z = position.z

        pc = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_data)
        pc = np.asarray(pc.tolist())[:,:3]
        pc_near = pc[(pc[:,0]**2 + pc[:,1]**2)< self.search_range**2]
        self.state = np.array([self.x, self.y, self.z])
        pc_near += self.state
        pc_high = pc_near[pc_near[:,2] > self.ground]
        pc_inrange = pc_high[pc_high[:,0] > self.search_area[0][0]]
        pc_inrange = pc_inrange[pc_inrange[:,0] < self.search_area[0][1]]
        pc_inrange = pc_inrange[pc_inrange[:,1] > self.search_area[1][0]]
        pc_inrange = pc_inrange[pc_inrange[:,1] < self.search_area[1][1]]
        print("gem_state:", self.state)
        print("shape:", pc_inrange.shape)
        print("relative averagex:", np.mean(pc_inrange[:,0]))
        with open("./src/Lidar_based_control/src/detection.csv", "a") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(pc_inrange[:, :2])



class Lidar_based_detector_2():

    def __init__(self):
        self.car = True

        self.search_range = 7
        self.car_range = 1.3
        self.ground = -1.5
        self.search_area = [[3,30],[-11,-1]]

        self.lon = 0.0
        self.lat = 0.0
        self.heading = 0.0
        self.olat = 40.0928563
        self.olon = -88.2359994
        self.offset = 0.46 

        # self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        # # self.sub_point_cloud = rospy.Subscriber('/lidar1/velodyne_points',PointCloud2, self.pointcloud_callback,queue_size=1)
        # self.sub_point_cloud = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback,queue_size=1)    # 仿真的的点云数据
        # # self.sub_point_cloud = message_filters.Subscriber('/lidar1/velodyne_points',PointCloud2)    # gem上面的点云数据
        if not self.car:
            self.sub_state = message_filters.Subscriber('/gazebo/model_states', ModelStates)
            self.sub_point_cloud = message_filters.Subscriber('/velodyne_points', PointCloud2)
            sync = message_filters.ApproximateTimeSynchronizer([self.sub_state, self.sub_point_cloud], 1, 0.1, allow_headerless=True)
        else:
            self.gnss_sub   = message_filters.Subscriber("/novatel/inspva", Inspva)
            self.sub_point_cloud = message_filters.Subscriber('/lidar1/velodyne_points',PointCloud2)
            sync = message_filters.ApproximateTimeSynchronizer([self.gnss_sub, self.sub_point_cloud], 1, 0.1, allow_headerless=True)
        
        sync.registerCallback(self.perception_multi_callback)

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

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   

    def get_gem_state(self):
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        curr_yaw = self.heading_to_yaw(self.heading) 
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)
        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)


    def perception_multi_callback(self, state_data, lidar_data):
        if not self.car:
            model_index = state_data.name.index("gem")
            position = state_data.pose[model_index].position
            self.x = position.x
            self.y = position.y
            self.z = position.z
        else:
            self.lat     = state_data.latitude  # latitude
            self.lon     = state_data.longitude # longitude
            self.heading = state_data.azimuth   # heading in degrees
            self.x, self.y, self.raw = self.get_gem_state()
            self.z = 0
            # print("yaw:", self.raw)
        
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_data)
        pc = np.asarray(pc.tolist())[:,:3]
        pc_near = pc[(pc[:,0]**2 + pc[:,1]**2)< self.search_range**2]
        # print("pc_near1:", pc_near)
        pc_near = pc_near[(pc_near[:,0]**2 + pc_near[:,1]**2) > self.car_range**2]
        # print("pc_near2:", pc_near)
        # print("relative averagez:", np.mean(pc_near[:,2]))
        # print("relative maxz:", np.max(pc_near[:,2]))
        # print("relative minz:", np.min(pc_near[:,2]))
        self.state = np.array([self.x, self.y, self.z])

        print("car state:", "x:", self.x, "y:", self.y, "raw:", self.raw)
        # pc_near = pc_near[pc_near[:,2] > self.ground]
        # print("pc1:", pc_near[0])
        # print("pc_near x:", np.average(pc_near[:,0]))
        # print("pc_near y:", np.average(pc_near[:,1]))
        pc_near[:, 0] = self.x + np.cos(self.raw) * pc_near[:,0] - np.sin(self.raw) * pc_near[:,1]  
        pc_near[:, 1] = self.y + np.sin(self.raw) * pc_near[:, 0] + np.cos(self.raw) * pc_near[:,1] 
        pc_near[:, 2] += self.z
        # print("pc1_transform:", pc_near[0])
        # pc_near += self.state
        pc_high = pc_near[pc_near[:,2] > self.ground]
        # # print("car state:", "x:", self.x, "y:", self.y)
        # print("pc_high(transform) x:", np.average(pc_high[:,0]))
        # print("pc_high(transform) y:", np.average(pc_high[:,1]))
        # print("pc_high shape", len(pc_high))
        pc_inrange = pc_high[pc_high[:,0] > self.search_area[0][0]]
        pc_inrange = pc_inrange[pc_inrange[:,0] < self.search_area[0][1]]
        pc_inrange = pc_inrange[pc_inrange[:,1] > self.search_area[1][0]]
        pc_inrange = pc_inrange[pc_inrange[:,1] < self.search_area[1][1]]
        print("pc_inrange shape", len(pc_inrange))
        # print("gem_state:", self.state)
        # print("shape:", pc_inrange.shape)
        # print("relative averagex:", np.mean(pc_inrange[:,0]))
        
        with open("/home/gem/demo_ws/src/Lidar_based_control/Lidar_based_control/src/detection.csv", "a") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(pc_inrange[:, :2])




if __name__ == '__main__':
    # # rospy.init_node("Lidar-perception")
    # # pub = rospy.Subscriber("che", String, pointcloud_callback,queue_size=1)
    # # rospy.spin()

    # rospy.init_node("hahaha")
    # sub = rospy.Subscriber("fds", box, pointcloud_callback, queue_size = 1)
    # rospy.spin()

    rospy.init_node('box-detector', anonymous=True)
    ppd = Lidar_based_detector_2()
    rospy.spin()
    # self.sub_point_cloud = rospy.Subscriber('/lidar1/velodyne_points',PointCloud2, self.pointcloud_callback,queue_size=1)