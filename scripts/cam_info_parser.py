
import rospy
import yaml
import cv2
import numpy as np
import time
from sensor_msgs.msg import CameraInfo

publisher_left = 0;
publisher_right = 0;
focal_length_FPGA = 0;

def callback(data):
    
    K = data.K
    global focal_length_FPGA 
    focal_length_FPGA = K[0]
    stamp = data.header.stamp
    camera_info_msg0.header.stamp = stamp
    camera_info_msg1.header.stamp = stamp
    publisher_left.publish(camera_info_msg0)
    publisher_right.publish(camera_info_msg1)

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing " +\
                                             "camera calibration data")
    arg_parser.add_argument("left_cam_nr", help="Number of left camera")
    args = arg_parser.parse_args()
    filename = args.filename
    cam_nr1 = int(args.left_cam_nr)
    cam_nr2 = cam_nr1 + 1

    publisher_left = rospy.Publisher("uvc_camera/cam_"+str(cam_nr1)+"/parsed_camera_info", CameraInfo, queue_size=10)
    publisher_right = rospy.Publisher("uvc_camera/cam_"+str(cam_nr2)+"/parsed_camera_info", CameraInfo, queue_size=10)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    rospy.Subscriber("/uvc_camera/cam_" + str(cam_nr1) +"/camera_info",CameraInfo,callback)
    camera_info_msg0 = CameraInfo()
    camera_info_msg1 = CameraInfo()

    """
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    left_camera_nr : int
	Number of the left camera in the stereo setup

    Returns
    -------
    camera_info_msg0 : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data of the left camera
    camera_info_msg1 : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data of the right camera
    """
    # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    D0 = np.array(calib_data["cam"+str(cam_nr1)]["distortion_coeffs"])
    D1 = np.array(calib_data["cam"+str(cam_nr2)]["distortion_coeffs"])
    K0 = np.array([[calib_data["cam"+str(cam_nr1)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr1)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr1)]["intrinsics"][1], calib_data["cam"+str(cam_nr1)]["intrinsics"][3]], [0, 0, 1]])    
    K1 = np.array([[calib_data["cam"+str(cam_nr2)]["intrinsics"][0], 0, calib_data["cam"+str(cam_nr2)]["intrinsics"][2]], [0, calib_data["cam"+str(cam_nr2)]["intrinsics"][1], calib_data["cam"+str(cam_nr2)]["intrinsics"][3]], [0, 0, 1]]) 
    res = np.array(calib_data["cam"+str(cam_nr1)]["resolution"])
    R = np.array([[calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][2]], [calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][0], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][1], calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][2]]]) 
    T = np.array([calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][0][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][1][3],calib_data["cam"+str(cam_nr2)]["T_cn_cnm1"][2][3]])


    # use opencv stereo rectify to calculate new intrinsics and procetion
    a = 1
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)

    #print "focal length FPGA" 

    rate = rospy.Rate(2000)

    # Run node and wait for message with FPGA focal length value
    while not rospy.is_shutdown():
         if (focal_length_FPGA > 0):
		break
         rate.sleep()
    #print focal_length_FPGA
    #print "start focal lenght"
    #print P1[0][0]

    while (focal_length_FPGA > P1[0][0]) and not rospy.is_shutdown():

    #apply a zoom of approx 30 pixels to new focal length to match with FPGA rectification
	a = a - 0.001
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K0, D0, K1, D1, (res[0],res[1]), R, T, alpha = a)
        rate.sleep()
        
    #print "final focal length"
    #print P1[0][0]
    
    # fill values in camera info messages
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

    rate = rospy.Rate(20)

    # Run node
    while not rospy.is_shutdown():
         rate.sleep()

