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
from sensor_msgs.msg import CameraInfo

publisher_left = 0;
publisher_right = 0;
focal_length_FPGA = 0;

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left_camera_index", required=True, help="Number of left camera")
    args = parser.parse_args(rospy.myargv()[1:])
    return args

def callback1(data):
    stamp = data.header.stamp
    id = data.header.frame_id
    camera_info_msg0.header.stamp = stamp
    camera_info_msg0.header.frame_id = id
    # fill values in camera info messages
    camera_info_msg0.width = data.width
    camera_info_msg0.height = data.height
    camera_info_msg0.D = data.D
    camera_info_msg0.K = data.K
    camera_info_msg0.R = data.R
    camera_info_msg0.P = data.P
    camera_info_msg0.distortion_model = "plumb_bob"
    publisher_left.publish(camera_info_msg0)

def callback2(data):
    stamp = data.header.stamp
    id = data.header.frame_id
    camera_info_msg1.header.stamp = stamp
    camera_info_msg1.header.frame_id = id
    # fill values in camera info messages
    camera_info_msg1.width = data.width
    camera_info_msg1.height = data.height
    camera_info_msg1.D = data.D
    camera_info_msg1.K = data.K
    camera_info_msg1.R = data.R
    Pnew = list(data.P)
    #swap sign of baseline x direction
    Pnew[3] = -Pnew[3]
    camera_info_msg1.P = tuple(Pnew)
    camera_info_msg1.distortion_model = "plumb_bob"
    publisher_right.publish(camera_info_msg1)


if __name__ == "__main__":

    # parse arguments from call
    args = parse_args()
    cam_nr1 = int(args.left_camera_index)
    cam_nr2 = cam_nr1 + 1

    publisher_left = rospy.Publisher("uvc_camera/cam_"+str(cam_nr1)+"/parsed_camera_info", CameraInfo, queue_size=10)
    publisher_right = rospy.Publisher("uvc_camera/cam_"+str(cam_nr2)+"/parsed_camera_info", CameraInfo, queue_size=10)

    # Initialize publisher node
    rospy.init_node('baseline_inverter', anonymous=True)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr1) +"/camera_info",CameraInfo,callback1)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr2) +"/camera_info",CameraInfo,callback2)
    camera_info_msg0 = CameraInfo()
    camera_info_msg1 = CameraInfo()

    # Run node and wait for callbacks
    rospy.spin()

