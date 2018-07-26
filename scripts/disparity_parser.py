#!/usr/bin/env python
#
# Author: Dominik Honegger <dominiho at inf.ethz.ch>
#
# Script to parse fpga disparity image messages present in Trimbot Dataset and
# publish new ROS compatible parsed dispariy messages
#
# Subscribes to uvc_camera/cam_x/image_depth and publishes new disparity
# message named uvc_camera/cam_x/disparity

import rospy
import yaml
import cv2
import numpy as np
import time
import argparse
import cv_bridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

rvstamp = 0;
focal_length_FPGA = 0;

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration_path", required=True, help="Path to yaml file containing " +\
                                             "camera calibration data")
    parser.add_argument("--left_camera_index", required=True, help="Number of left camera")
    args = parser.parse_args(rospy.myargv()[1:])
    return args

def callback(data):
    global rvstamp 
    rvstamp = data.header.stamp
    K = data.K
    P = data.P
    global focal_length_FPGA 
    #check if P matrix has valid value as in new datasets
    #otherwise take focallength from K matrix (for old datasets)
    if(P[0] > 0):
	focal_length_FPGA = P[0]
    else:
        focal_length_FPGA = K[0]


def callbackimage(data):
    #take timestamp from info messages
    #stamp = data.header.stamp
    disparity_msg.header.stamp = rvstamp
    disparity_msg.header.frame_id = "cam_" + str(cam_nr1) + "_optical_frame"
    disparity_msg.image.header.stamp = rvstamp
    disparity_msg.image.header.frame_id = "cam_" + str(cam_nr1) + "_optical_frame"

    bridge = CvBridge()
    testframe = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #testframe = testframe >> 3
    floatframe = cv2.normalize((testframe).astype('float32'),None,alpha = 0,beta = 31,norm_type=cv2.NORM_MINMAX,dtype = cv2.CV_32F)
    floatframe = floatframe - 2.0
    #print floatframe

    disparity_msg.image = bridge.cv2_to_imgmsg(floatframe, "32FC1")
    publisher_disparity.publish(disparity_msg)

if __name__ == "__main__":

    # parse arguments from call
    args = parse_args()
    filename = args.calibration_path
    cam_nr1 = int(args.left_camera_index)
    cam_nr2 = cam_nr1 + 1

    publisher_disparity = rospy.Publisher("uvc_camera/cam_"+str(cam_nr1)+"/disparity", DisparityImage, queue_size=10)

    # Initialize publisher node
    rospy.init_node('disparity_parser', anonymous=True)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr1) +"/camera_info",CameraInfo,callback)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr1) +"/image_depth",Image,callbackimage)
    camera_info_msg0 = CameraInfo()
    camera_info_msg1 = CameraInfo()
    disparity_msg = DisparityImage()

    # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse calibration.yaml file to get intrinsic and extrinsic params of stereo pair
    D0 = np.array(calib_data["cam"+str(cam_nr1)]["distortion_coeffs"])
    D1 = np.array(calib_data["cam"+str(cam_nr2)]["distortion_coeffs"])
    K0 = np.array([[calib_data["cam"+str(cam_nr1)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr1)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr1)]["intrinsics"][1], calib_data["cam"+str(cam_nr1)]["intrinsics"][3]], [0, 0, 1]])    
    K1 = np.array([[calib_data["cam"+str(cam_nr2)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr2)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr2)]["intrinsics"][1], calib_data["cam"+str(cam_nr2)]["intrinsics"][3]], [0, 0, 1]]) 
    res = np.array(calib_data["cam"+str(cam_nr1)]["resolution"])
    R = np.array([[calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][2]]]) 
    T = np.array([calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][3]])

    # use opencv stereo rectify to calculate new intrinsics and projection
    a = 1.0
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)

    rate = rospy.Rate(2000)

    # Run node and wait for message with FPGA focal length value
    while not rospy.is_shutdown():
         if (focal_length_FPGA > 0):
		break
         rate.sleep()
    # change alpha parameter (zoom) to achieve similar focal length than FPGA
    while (focal_length_FPGA > P1[0][0]) and not rospy.is_shutdown():
        if(a < 0.001 ):
            print "error finding matching focallength"
            break
	a = a - 0.001
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)
        rate.sleep() 

    print "left cam number:", cam_nr1, "focal length fpga:", focal_length_FPGA, "focal length ROS:", P1[0][0]

    # fill values in camera info messages
    disparity_msg.image.width = res[0]
    disparity_msg.image.height = res[1]
    disparity_msg.image.encoding = "32FC1"
    disparity_msg.image.is_bigendian = 0
    disparity_msg.image.step = res[0]*4
    disparity_msg.f = P1[0][0]
    disparity_msg.T = -calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][3]
    disparity_msg.min_disparity = 0.0
    disparity_msg.max_disparity = 31.0
    disparity_msg.delta_d = 1.0

    camera_info_msg0.width = res[0]
    camera_info_msg0.height = res[1]
    camera_info_msg0.D = D0
    camera_info_msg0.K = K0.reshape(9,1)
    camera_info_msg0.R = R1.reshape(9,1)
    camera_info_msg0.P = P1.reshape(12,1)
    camera_info_msg0.distortion_model = "plumb_bob"

    camera_info_msg1.width = res[0]
    camera_info_msg1.height = res[1]
    camera_info_msg1.D = D1
    camera_info_msg1.K = K1.reshape(9,1)
    camera_info_msg1.R = R2.reshape(9,1)
    camera_info_msg1.P = P2.reshape(12,1)
    camera_info_msg1.distortion_model = "plumb_bob"

    # Run node and wait for callbacks
    rospy.spin()

