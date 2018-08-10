#!/usr/bin/env python

import math
import rospy
import sys
import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import struct
import argparse
import numpy as np

camera ={}
imageOverlay = {}
cameraInfo = CameraInfo()
cameraModel = PinholeCameraModel()
bridge = cv_bridge.CvBridge()
tf_ = tf.TransformListener()
velodyneData = []

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera_disp_topic", required=True, help="disparity image topics")
    parser.add_argument("--camera_info_topic", required=True, help="disparity info topics")
    parser.add_argument("--velodyne_topic", required=True, help="velodyne scanner topics")
    args = parser.parse_args(rospy.myargv()[1:])
    return args

def info_callback(data):
	global cameraModel, camera
	cameraInfo = data
        #use camera info params to fill camera model
	cameraModel.fromCameraInfo( cameraInfo )

def velodyne_callback(data):
	global velodyneData

	formatString = 'fff'
	if data.is_bigendian:
		formatString = '>' + formatString
	else:
		formatString = '<' + formatString

	points = []
        #first 12 bytes contain x, y, z data from velodyne scanner

	for index in range( 0, len( data.data ), 32 ):
		points.append( struct.unpack( formatString, data.data[ index:index + 12 ] ) )
	velodyneData = points

def dispimage_callback(data):
	global velodyneData, bridge, tf_
	cv_image = {}

	try:
		cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
	except cv_bridge.CvBridgeError as e:
			print('Failed to convert image', e)
			return

        #shift depth values to red channel only, dismiss other channels
        for i in range(0,data.height-1):
         for j in range (0,data.width-1):
           cv_image[i][j] = [0,0,cv_image[i,j,0]] 

	# the transform is same for every frame 
	(trans, rot) = tf_.lookupTransform( 'cam_0_optical_frame', 'velodyne_link', rospy.Time( 0 ) )

	# print("transformation: ", trans, rot)
	trans = tuple(trans) + ( 1,  )
	# print(trans)
	rotationMatrix = tf.transformations.quaternion_matrix( rot )
	# append translation to the last column of rotation matrix(4x4)
	rotationMatrix[ :, 3 ] = trans
	# print('rotationMatrix::  ', rotationMatrix)

	if velodyneData:
                # print len(velodyneData)
		for i in range(0, len(velodyneData) - 1):
			try:
				# converting to homogeneous coordinates
				point = [velodyneData[i][0], velodyneData[i][1], velodyneData[i][2],1]

			except IndexError:
				print("Index Error!!!!!")
				break

			#  project 3D point to 2D uv 
			rotatedPoint = rotationMatrix.dot( point )
			uv = cameraModel.project3dToPixel( rotatedPoint )

			distance = math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) )
                        disparity = 540*0.03/distance

			# check if the uv point is valid
			if uv[0] >= 0 and uv[0] <= data.width and uv[1] >= 0 and uv[1] <= data.height and rotatedPoint[2] > 0:
				# prevent existing disparity value in red channel from overwriting
                                valuetorestore = int(cv_image[int(uv[1]),int(uv[0]),2])

                                #print 2x2 patches
				#cv2.line(cv_image,(int( uv[0] ),int( uv[1] )),(int( uv[0] )+2,int( uv[1] ) +2),(disparity*8,0,valuetorestore),3)

			 	#print velodyne range in disparity scale at pixel level accuracy
				cv_image[int(uv[1])][int(uv[0])] = [disparity*8,0,valuetorestore]

	try:
		imageOverlay.publish(bridge.cv2_to_imgmsg( cv_image, 'bgr8' ) )
	except cv_bridge.CvBridgeError as e:
		print( 'Failed to convert image', e )
		return


if __name__ == '__main__':
	try:
	        # parse arguments from call
	        args = parse_args()
	        camerainfoName = args.camera_info_topic
                imagedispName = args.camera_disp_topic
	        velodyneName = args.velodyne_topic

		# Initialize publisher node#
		rospy.init_node('velodyne_disparity_image')

		# Subscribe to topic, cameraInfo and callback function.
		camerainfo = rospy.Subscriber( camerainfoName, CameraInfo, callback = info_callback)
		disparityimage = rospy.Subscriber(imagedispName, Image, callback = dispimage_callback)
		velodynePoint = rospy.Subscriber(velodyneName, PointCloud2, callback = velodyne_callback)

		# look up lidar overlay image
		imageOverlayName = rospy.resolve_name( 'velodyne_disparity_overlay')

		# Publish the lidar overlay image
		imageOverlay = rospy.Publisher( imageOverlayName, Image, queue_size = 1)

		rospy.spin()

	except rospy.ROSInterruptException: pass
