#!/usr/bin/env python
#
# Author: Dominik Honegger <dominiho at inf.ethz.ch>
#
# Script to parse camera_info messages present in Trimbot Dataset and
# publish new ROS compatible parsed_info_messages
#
# Subscribes to uvc_camera/cam_x/camera_info and publishes new camera_info
# message names uvc_camera/cam_x/parsed_camera_info

import rospy
import yaml
import cv2
import numpy as np
import time
import argparse
from sensor_msgs.msg import Image

publisher_left = 0;

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left_camera_index", required=True, help="Number of left camera")
    args = parser.parse_args(rospy.myargv()[1:])
    return args

def callback1(data):
    stamp = data.header.stamp
    id = data.header.frame_id
    image_msg0.header.stamp = stamp
    image_msg0.header.frame_id = "cam_0_optical_frame"
    image_msg0.data = data.data
    image_msg0.height = data.height
    image_msg0.width = data.width
    image_msg0.encoding = data.encoding
    image_msg0.step = data.step
    publisher_left.publish(image_msg0)



if __name__ == "__main__":

    # parse arguments from call
    args = parse_args()
    cam_nr1 = int(args.left_camera_index)
    cam_nr2 = cam_nr1 + 10

    publisher_left = rospy.Publisher("uvc_camera/cam_"+str(cam_nr2)+"/parsed_image_rect", Image, queue_size=10)

    # Initialize publisher node
    rospy.init_node('baseline_inverter', anonymous=True)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr1) +"/image_rect",Image,callback1)
    #rospy.Subscriber("/pair"+ str(cam_nr1) +"/left/image_rect_color",Image,callback1)
    image_msg0 = Image()

    # Run node and wait for callbacks
    rospy.spin()

