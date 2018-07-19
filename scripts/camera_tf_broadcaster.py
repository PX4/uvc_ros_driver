#!/usr/bin/env python
#
# Author: Dominik Honegger <dominiho at inf.ethz.ch>
#
# Script to parse extrinsic camera calibration values present in Trimbot Dataset and
# publish ROS tf camera frames
#

import rospy
import yaml
import cv2
import numpy as np
import time
import argparse
import tf
import tf2_ros
import geometry_msgs.msg
import numpy
from numpy.linalg import inv
from sensor_msgs.msg import CameraInfo

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration_path", required=True, help="Path to yaml file containing " +\
                                             "camera calibration data")
    args = parser.parse_args(rospy.myargv()[1:])
    return args


if __name__ == "__main__":

    # parse arguments from call
    args = parse_args()
    filename = args.calibration_path

    # Initialize publisher node
    rospy.init_node('camera_tf_broadcaster', anonymous=True)


  # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse calibration.yaml file to get intrinsic and extrinsic params of stereo pair
 
    Tcnm1 = calib_data["cam1"]["T_cn_cnm1"]
    Tcnm2 = calib_data["cam2"]["T_cn_cnm1"]
    Tcnm3 = calib_data["cam3"]["T_cn_cnm1"]
    Tcnm4 = calib_data["cam4"]["T_cn_cnm1"]
    Tcnm5 = calib_data["cam5"]["T_cn_cnm1"]
    Tcnm6 = calib_data["cam6"]["T_cn_cnm1"]
    Tcnm7 = calib_data["cam7"]["T_cn_cnm1"]
    Tcnm8 = calib_data["cam8"]["T_cn_cnm1"]
    Tcnm9 = calib_data["cam9"]["T_cn_cnm1"]

    #print Tcnm1

    #transformations of left cameras are T2temp,T4temp,T6temp,T8temp
    T2temp = numpy.matmul([Tcnm2],[Tcnm1])
    T3temp = numpy.matmul([Tcnm3],T2temp)
    T4temp = numpy.matmul([Tcnm4],T3temp)
    T5temp = numpy.matmul([Tcnm5],T4temp)
    T6temp = numpy.matmul([Tcnm6],T5temp)
    T7temp = numpy.matmul([Tcnm7],T6temp)
    T8temp = numpy.matmul([Tcnm8],T7temp)
    T9temp = numpy.matmul([Tcnm9],T8temp)

    cam_nr1 = 2
    cam_nr2 = 3
    D0 = np.array(calib_data["cam"+str(cam_nr1)]["distortion_coeffs"])
    D1 = np.array(calib_data["cam"+str(cam_nr2)]["distortion_coeffs"])
    K0 = np.array([[calib_data["cam"+str(cam_nr1)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr1)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr1)]["intrinsics"][1], calib_data["cam"+str(cam_nr1)]["intrinsics"][3]], [0, 0, 1]])    
    K1 = np.array([[calib_data["cam"+str(cam_nr2)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr2)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr2)]["intrinsics"][1], calib_data["cam"+str(cam_nr2)]["intrinsics"][3]], [0, 0, 1]]) 
    res = np.array(calib_data["cam"+str(cam_nr1)]["resolution"])
    R = np.array([[calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][2]]]) 
    T = np.array([calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][3]])
    # use opencv stereo rectify to calculate new intrinsics and projection
    a = 1
    R2, R3, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)

    cam_nr1 = 4
    cam_nr2 = 5
    D0 = np.array(calib_data["cam"+str(cam_nr1)]["distortion_coeffs"])
    D1 = np.array(calib_data["cam"+str(cam_nr2)]["distortion_coeffs"])
    K0 = np.array([[calib_data["cam"+str(cam_nr1)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr1)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr1)]["intrinsics"][1], calib_data["cam"+str(cam_nr1)]["intrinsics"][3]], [0, 0, 1]])    
    K1 = np.array([[calib_data["cam"+str(cam_nr2)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr2)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr2)]["intrinsics"][1], calib_data["cam"+str(cam_nr2)]["intrinsics"][3]], [0, 0, 1]]) 
    res = np.array(calib_data["cam"+str(cam_nr1)]["resolution"])
    R = np.array([[calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][2]]]) 
    T = np.array([calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][3]])
    # use opencv stereo rectify to calculate new intrinsics and projection
    a = 1
    R4, R5, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)
    #print R1
    #extend to 4x4 matrix
    R2 = np.append(R2,[[0,0,0]],0)
    R2 = np.append(R2,[[0],[0],[0],[1]],1)
    R3 = np.append(R3,[[0,0,0]],0)
    R3 = np.append(R3,[[0],[0],[0],[1]],1)
    R4 = np.append(R4,[[0,0,0]],0)
    R4 = np.append(R4,[[0],[0],[0],[1]],1)
    R5 = np.append(R5,[[0,0,0]],0)
    R5 = np.append(R5,[[0],[0],[0],[1]],1)

    #T2temp = numpy.matmul(R1,T2temptest)
    #print inv(T2temp)

    T2 = np.matrix([[T2temp[0][0][0],T2temp[0][0][1],T2temp[0][0][2],T2temp[0][0][3]],[T2temp[0][1][0],T2temp[0][1][1],T2temp[0][1][2],T2temp[0][1][3]],[T2temp[0][2][0],T2temp[0][2][1],T2temp[0][2][2],T2temp[0][2][3]],[T2temp[0][3][0],T2temp[0][3][1],T2temp[0][3][2],T2temp[0][3][3]]])
    #print R2
    #print R5
    #print T2*R2
    T2corr = inv(R2*T2)
    T2 = inv(T2)

    T4 = np.matrix([[T4temp[0][0][0],T4temp[0][0][1],T4temp[0][0][2],T4temp[0][0][3]],[T4temp[0][1][0],T4temp[0][1][1],T4temp[0][1][2],T4temp[0][1][3]],[T4temp[0][2][0],T4temp[0][2][1],T4temp[0][2][2],T4temp[0][2][3]],[T4temp[0][3][0],T4temp[0][3][1],T4temp[0][3][2],T4temp[0][3][3]]])
    T4 = inv(T4)

    T6 = np.matrix([[T6temp[0][0][0],T6temp[0][0][1],T6temp[0][0][2],T6temp[0][0][3]],[T6temp[0][1][0],T6temp[0][1][1],T6temp[0][1][2],T6temp[0][1][3]],[T6temp[0][2][0],T6temp[0][2][1],T6temp[0][2][2],T6temp[0][2][3]],[T6temp[0][3][0],T6temp[0][3][1],T6temp[0][3][2],T6temp[0][3][3]]])
    T6 = inv(T6)

    T8 = np.matrix([[T8temp[0][0][0],T8temp[0][0][1],T8temp[0][0][2],T8temp[0][0][3]],[T8temp[0][1][0],T8temp[0][1][1],T8temp[0][1][2],T8temp[0][1][3]],[T8temp[0][2][0],T8temp[0][2][1],T8temp[0][2][2],T8temp[0][2][3]],[T8temp[0][3][0],T8temp[0][3][1],T8temp[0][3][2],T8temp[0][3][3]]])
    T8 = inv(T8)


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

	    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
	    broadcaster2corr = tf2_ros.StaticTransformBroadcaster()
	    broadcaster4 = tf2_ros.StaticTransformBroadcaster()
	    broadcaster6 = tf2_ros.StaticTransformBroadcaster()
	    broadcaster8 = tf2_ros.StaticTransformBroadcaster()

	    static_transformStamped2 = geometry_msgs.msg.TransformStamped()
	    static_transformStamped2.header.stamp = rospy.Time.now()
	    static_transformStamped2.header.frame_id = "cam_0_optical_frame"
	    static_transformStamped2.child_frame_id = "cam_2_optical_frame"
	    static_transformStamped2.transform.translation.x = T2[0,3]
	    static_transformStamped2.transform.translation.y = T2[1,3]
	    static_transformStamped2.transform.translation.z = T2[2,3]
	    quad = tf.transformations.quaternion_from_matrix(T2)
	    static_transformStamped2.transform.rotation.x = quad[0]
	    static_transformStamped2.transform.rotation.y = quad[1]
	    static_transformStamped2.transform.rotation.z = quad[2]
	    static_transformStamped2.transform.rotation.w = quad[3]

	    broadcaster2.sendTransform(static_transformStamped2)

	    static_transformStamped2corr = geometry_msgs.msg.TransformStamped()
	    static_transformStamped2corr.header.stamp = rospy.Time.now()
	    static_transformStamped2corr.header.frame_id = "cam_0_optical_frame"
	    static_transformStamped2corr.child_frame_id = "cam_2_optical_frame_corr"
	    static_transformStamped2corr.transform.translation.x = T2corr[0,3]
	    static_transformStamped2corr.transform.translation.y = T2corr[1,3]
	    static_transformStamped2corr.transform.translation.z = T2corr[2,3]
	    quad = tf.transformations.quaternion_from_matrix(T2corr)
	    static_transformStamped2corr.transform.rotation.x = quad[0]
	    static_transformStamped2corr.transform.rotation.y = quad[1]
	    static_transformStamped2corr.transform.rotation.z = quad[2]
	    static_transformStamped2corr.transform.rotation.w = quad[3]

	    broadcaster2corr.sendTransform(static_transformStamped2corr)

	    static_transformStamped4 = geometry_msgs.msg.TransformStamped()
	    static_transformStamped4.header.stamp = rospy.Time.now()
	    static_transformStamped4.header.frame_id = "cam_0_optical_frame"
	    static_transformStamped4.child_frame_id = "cam_4_optical_frame"
	    static_transformStamped4.transform.translation.x = T4[0,3]
	    static_transformStamped4.transform.translation.y = T4[1,3]
	    static_transformStamped4.transform.translation.z = T4[2,3]
	    quad = tf.transformations.quaternion_from_matrix(T4)
	    static_transformStamped4.transform.rotation.x = quad[0]
	    static_transformStamped4.transform.rotation.y = quad[1]
	    static_transformStamped4.transform.rotation.z = quad[2]
	    static_transformStamped4.transform.rotation.w = quad[3]

	    broadcaster4.sendTransform(static_transformStamped4)

	    static_transformStamped6 = geometry_msgs.msg.TransformStamped()
	    static_transformStamped6.header.stamp = rospy.Time.now()
	    static_transformStamped6.header.frame_id = "cam_0_optical_frame"
	    static_transformStamped6.child_frame_id = "cam_6_optical_frame"
	    static_transformStamped6.transform.translation.x = T6[0,3]
	    static_transformStamped6.transform.translation.y = T6[1,3]
	    static_transformStamped6.transform.translation.z = T6[2,3]
	    quad = tf.transformations.quaternion_from_matrix(T6)
	    static_transformStamped6.transform.rotation.x = quad[0]
	    static_transformStamped6.transform.rotation.y = quad[1]
	    static_transformStamped6.transform.rotation.z = quad[2]
	    static_transformStamped6.transform.rotation.w = quad[3]

	    broadcaster6.sendTransform(static_transformStamped6)

	    static_transformStamped8 = geometry_msgs.msg.TransformStamped()
	    static_transformStamped8.header.stamp = rospy.Time.now()
	    static_transformStamped8.header.frame_id = "cam_0_optical_frame"
	    static_transformStamped8.child_frame_id = "cam_8_optical_frame"
	    static_transformStamped8.transform.translation.x = T8[0,3]
	    static_transformStamped8.transform.translation.y = T8[1,3]
	    static_transformStamped8.transform.translation.z = T8[2,3]
	    quad = tf.transformations.quaternion_from_matrix(T8)
	    static_transformStamped8.transform.rotation.x = quad[0]
	    static_transformStamped8.transform.rotation.y = quad[1]
	    static_transformStamped8.transform.rotation.z = quad[2]
	    static_transformStamped8.transform.rotation.w = quad[3]

	    broadcaster8.sendTransform(static_transformStamped8)

            rate.sleep()

    # Run node and wait for callbacks
    rospy.spin()

