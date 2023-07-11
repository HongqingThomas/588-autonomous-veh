#! /usr/bin/env python
from __future__ import print_function
import sys
# souce_path = "./src/gem_vision/camera_vision/scripts/" # for simulator
souce_path = "./src/vehicle_drivers/gem_vision/gem_vision/camera_vision/scripts/" # for real car

# sys.path can be used to add additional directories to the Python module search path, allowing ROS nodes to import modules or packages from custom locations.
# To be specify, if you want to import a function from some module you write, you need to add its path by sys.path.append()
sys.path.append(souce_path)
from camera_utils import *
sys.path.append(souce_path + "Detector/")
from yolo_detect_image import yolo_detect_image
# sys.path.append(souce_path + "lane_detect/")
# from lane_detector import lane_detector

object_detection = True
lane_detection = False

Simulator = True
Real_car = False

class ImageConverter:
    def __init__(self):
        self.node_name = "gem_vision"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup) # this function will be called once when the node is shut down
        self.bridge = CvBridge() # By using CvBridge(), ROS nodes that process images can easily convert between ROS image messages and OpenCV images, enabling them to use the full range of computer vision functionality provided by OpenCV.
        # self.last_state_info = [[], []]
        # self.last_ssim_info = [deque(), deque()]
        self.frame_counter = 0
        print("initworks!")

        # Subscribe camera rgb and depth information
        # In ROS, it is often necessary to process multiple streams of data that are timestamped differently, such as sensor measurements and control commands. 
        # To ensure that the data is processed in a consistent and synchronized manner, the message_filters package provides a set of classes that allow nodes to subscribe to multiple topics and receive synchronized messages.
        # 1. rospy.get_param() allows a ROS node to retrieve a parameter value from the ROS parameter server.
        # 2. message_filters.Subscriber() allows you to subscribe to multiple topics and reveive synchronized message in a callback function
        #    ApproximateTimeSynchronizer() is used for synchronized, queue_size specifies how many messages can be buffered before starts to drop messages and slop specifies the allowed time difference between message.
        #    registerCallback() is called with the synchronized messages as arguments whenever a new set of messages is recieved.
        
        # for real car
        if Real_car:
            depth_img_topic = rospy.get_param('depth_info_topic','/zed2/zed_node/depth/depth_registered')
            self.depth_img_sub = message_filters.Subscriber(depth_img_topic,Image)
            self.subcriber_rgb = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
            self.subcriber_rgb_camera = message_filters.Subscriber('/zed2/zed_node/rgb_raw/camera_info', CameraInfo)
            sync = message_filters.ApproximateTimeSynchronizer([self.subcriber_rgb, self.depth_img_sub, self.subcriber_rgb_camera], 10, 1)
            sync.registerCallback(self.car_multi_callback)
        
        # for simulator
        elif Simulator:
            self.subcriber_rgb = message_filters.Subscriber('/front_single_camera/image_raw', Image)
            sync = self.subcriber_rgb
            sync.registerCallback(self.simulator_single_callback)

        # Publish Boudingbox information of objects
        self.image_pub = rospy.Publisher("/object_detection", Detected_msg, queue_size=1)
        self.image_pub_rviz = rospy.Publisher("/object_detection_person", Image, queue_size=1)

    # for real car
    def car_multi_callback(self, rgb, depth, camera_info):
        try:
            # print("multi_callback works!")
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        out_frame = rgb_frame
        detectBox = Detected_msg()
        # ----------------- Imaging processing code starts here ----------------\
        # # Object Detection
        if object_detection == True:

            # input: out_frame -> cv2(rgb_img), path; 
            # output: detected_list -> [left, top, left + width, top + height, classIds[i], confidence] for all detect_box with target classfied label
            #         out_frame -> cv2(rgb_img) with Draw a prediction box with confidence and title
            detected_list, out_frame = yolo_detect_image(out_frame, souce_path) 

            # output: object_camera_coordinate_list -> [distance, camera_x, camera_y, classId, confidence, camera_z] for each detected_list
            #         out_frame-> cv2(rgb_img) with Draw a prediction box with confidence, title and distance
            
            object_camera_coordinate_list, out_frame = calculate_object_distance(detected_list, depth_frame, camera_info, out_frame)
            
            # Add information you want to publish
            # for i in  range(len(detected_list)):
            if len(object_camera_coordinate_list) > 0:
                detectBox.middle_y = object_camera_coordinate_list[0][0]
            else:
                detectBox.middle_y = 10000
            print("object_camera_coordinate_list:", object_camera_coordinate_list)
        # cv2imshow(self, out_frame, "out_frame", 1)
        # cvImg = out_frame

        # cv2imshow(out_frame, "out_frame", 1)
        # ----------------------------------------------------------------------
        self.image_pub.publish(detectBox)
        cv_img = CvBridge().cv2_to_imgmsg(out_frame, "bgr8")
        self.image_pub_rviz.publish(cv_img)

    # for simulator
    def simulator_single_callback(self, rgb):
        try:
            print("single_callback works!")
            # rgb_frame = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            rgb_frame = cv2.imread(souce_path + "human_picture.jpeg")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        out_frame = rgb_frame
        detectBox = Detected_msg()
        # ----------------- Imaging processing code starts here ----------------\
        # # Object Detection
        if object_detection == True:

            # input: out_frame -> cv2(rgb_img), path; 
            # output: detected_list -> [left, top, left + width, top + height, classIds[i], confidence] for all detect_box with target classfied label
            #         out_frame -> cv2(rgb_img) with Draw a prediction box with confidence and title
            detected_list, out_frame = yolo_detect_image(out_frame, souce_path) 
            print("detected_list:", detected_list)
            # output: object_camera_coordinate_list -> [distance, camera_x, camera_y, classId, confidence, camera_z] for each detected_list
            #         out_frame-> cv2(rgb_img) with Draw a prediction box with confidence, title and distance
            # object_camera_coordinate_list, out_frame = calculate_object_distance(detected_list, depth_frame, camera_info, out_frame)
            # Add information you want to publish
            # for i in  range(len(detected_list)):
            # if len(object_camera_coordinate_list) > 0:
            #     detectBox.middle_y = object_camera_coordinate_list[0][0]
            # else:
            #     detectBox.middle_y = 10000
            # print("object_camera_coordinate_list:", object_camera_coordinate_list)
            # detectBox.object_distance.append(object_camera_coordinate_list[0][0])
            # detectBox.object_x.append(object_camera_cordinate_list[i][1])
            # detectBox.object_y.append(object_camera_coordinate_list[i][2])
            # detectBox.classId.append(object_camera_coordinate_list[i][3])
            # detectBox.confidence.append(object_camera_coordinate_list[i][4])
        # cv2imshow(self, out_frame, "out_frame", 1)
        # cvImg = out_frame

        cv2imshow(out_frame, "out_frame", 1)
        # ----------------------------------------------------------------------
        self.image_pub.publish(detectBox)
        cv_img = CvBridge().cv2_to_imgmsg(out_frame, "bgr8")
        self.image_pub_rviz.publish(cv_img)


    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()

def main(args):
    
    # In ROS, nodes are typically event-driven and are designed to respond to incoming messages or events. 
    # In order to receive and process these messages, ROS nodes register callback functions that are executed when a new message is received on a subscribed topic. 
    # The rospy.spin() function is used to continuously process these callbacks and wait for new messages on subscribed topics.

    # When rospy.spin() is called, the ROS node enters into a loop that runs until the node is shutdown. 
    # During each iteration of the loop, rospy.spin() checks for new messages on subscribed topics and calls any registered callbacks to process the messages. 
    # If no new messages are available, the function blocks and waits until a new message arrives.

    # The rospy.spin() function is typically called at the end of a ROS node's main function after all publishers and subscribers have been initialized. 
    # This allows the node to continuously process incoming messages and respond to events until it is shutdown.
    
    try:
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.destryAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    