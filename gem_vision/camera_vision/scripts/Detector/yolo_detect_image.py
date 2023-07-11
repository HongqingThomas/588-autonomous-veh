import cv2
import numpy as np
import argparse
from utils import *

def yolo_detect_image(image_frame, souce_path):
    source_path = souce_path + "Detector/"
    # Detect targets, whithin class of COCO dataset
    target = "person"
    # Define constants. CONF_THRESHOLD is confidence threshold. Only detection with confidence greater than this will be retained. NMS_THRESHOLD is used for non-max suppression
    CONF_THRESHOLD = 0.3
    NMS_THRESHOLD = 0.4
    # Create blob from image
    
    #image pretrain
    blob = cv2.dnn.blobFromImage(image_frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    
    # Read COCO dataset classes
    with open(source_path + 'coco.txt', 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')

    # Load the network with YOLOv3 weights and config using darknet framework
    net = cv2.dnn.readNet(source_path + "yolov3.weights", source_path + "yolov3.cfg", "darknet")
    
    # Get the output layer names used for forward pass
    out_names = net.getUnconnectedOutLayersNames()
    # Set the input
    net.setInput(blob)
    # Run forward pass
    outs = net.forward(out_names) # the output of net.forward() is a list of numpy arrays, with each array representing the output from a single ouput layer.
    # Process output and draw predictions
    detected_list, bbx_frame = process_frame(image_frame, outs, classes, CONF_THRESHOLD, NMS_THRESHOLD, target)
    return detected_list, bbx_frame