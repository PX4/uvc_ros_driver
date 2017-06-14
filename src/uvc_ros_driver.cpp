/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * uvc_ros_driver.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: nicolas, christoph, simone
 *
 *  The code below is based on the example provided at
 *  https://int80k.com/libuvc/doc/
 */

#include "uvc_ros_driver.h"

#include <functional>
#include <iostream>

namespace uvc
{

static void callback(uvc_frame *frame, void *arg)
{
	uvcROSDriver *obj = (uvcROSDriver *)arg;
	obj->uvc_cb(frame);
}

static bool myPairMax(std::pair<int, int> p, std::pair<int, int> p1)
{
	// gratest value is supposed be on the second index
	return p.second < p1.second;
}

uvcROSDriver::~uvcROSDriver()
{
	if(serial_port_open_){
		setParam("CAMERA_ENABLE", 0.0f);
	}
	mavlink_message_t message;
	mavlink_param_value_t param;
	mavlink_heartbeat_t heartbeat;
	int count = 0;
	bool wait=1;
	std::string str = "CAMERA_ENABLE";

	if(serial_port_open_){
		while(wait){
			int res = sp_.read_message(message);
			if(res == -1){
				serial_port_open_ = false;
				break;
			}
			if(res != 0){
				if(message.msgid==22){
					mavlink_msg_param_value_decode(&message, &param);
					if(str.compare(param.param_id)==0 && param.param_value==0){
						wait = 0;
					}
					else{
						setParam("CAMERA_ENABLE", 0.0f);
					}
					//std::cout << "received id " << param.param_id << " value " << param.param_value <<" iteration " 				<< (float) count << std::endl;
					count++;
				}
			}
		}

	printf("Camera Disabled \n");
	// close serial port
	sp_.close_serial();
	usleep(500000);
	uvc_stop_streaming(devh_);

	// close uvc device
	uvc_close(devh_);
	printf("Device closed");
	// ROS_INFO("Device closed");
	uvc_unref_device(dev_);
	uvc_exit(ctx_);
	}
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::initDevice()
{
	// initialize serial port
	// sp_ = Serial_Port("/dev/ttyUSB0", 115200);
	sp_ = Serial_Port("/dev/serial/by-id/usb-Cypress_FX3-if02", 115200);

        bool first_fault = true;
	int open = 0;
        while (true)
        {
        	ros::spinOnce();
		if(!nh_.ok())
			return;
        	open = sp_.open_serial();

        	if (open != -1){
			serial_port_open_ = true;
          		break;
		}
       		if (first_fault)
        	{
          		ROS_ERROR("Couldn't open serialport /dev/serial/by-id/usb-Cypress_FX3-if02. Will retry every second.");
          		first_fault = false;
       		 }
        	sleep(1.0);
        }

	//sp_.open_serial();

	// initialize camera image publisher
	switch (n_cameras_) {
	case 10:
		cam_9_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_9/image_raw", 5);
		cam_9_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_9/camera_info", 5);
		cam_8c_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_8/image_rect", 5);
		cam_8c_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_8/camera_info", 5);
		cam_8d_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_8/image_depth", 5);
		cam_8d_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_8/camera_info", 5);

	case 9:
		cam_8_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_8/image_raw", 5);
		cam_8_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_8/camera_info", 5);

	case 8:
		cam_7_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_7/image_raw", 5);
		cam_7_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_7/camera_info", 5);
		cam_6c_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_6/image_rect", 5);
		cam_6c_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_6/camera_info", 5);
		cam_6d_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_6/image_depth", 5);
		cam_6d_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_6/camera_info", 5);

	case 7:
		cam_6_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_6/image_raw", 5);
		cam_6_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_6/camera_info", 5);

	case 6:
		cam_5_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_5/image_raw", 5);
		cam_5_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_5/camera_info", 5);
		cam_4c_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_4/image_rect", 5);
		cam_4c_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_4/camera_info", 5);
		cam_4d_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_4/image_depth", 5);
		cam_4d_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_4/camera_info", 5);

	case 5:
		cam_4_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_4/image_raw", 5);
		cam_4_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_4/camera_info", 5);

	case 4:
		cam_3_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_3/image_raw", 5);
		cam_3_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_3/camera_info", 5);
		cam_2c_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_2/image_rect", 5);
		cam_2c_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_2/camera_info", 5);
		cam_2d_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_2/image_depth", 5);
		cam_2d_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_2/camera_info", 5);

	case 3:
		cam_2_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_2/image_raw", 5);
		cam_2_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_2/camera_info", 5);

	case 2:
		cam_1_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_1/image_raw", 5);
		cam_1_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_1/camera_info", 5);
		cam_0c_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_0/image_rect", 5);
		cam_0c_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_0/camera_info", 5);
		cam_0d_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_0/image_depth", 5);
		cam_0d_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_0/camera_info", 5);

	default:
		cam_0_pub_ = nh_.advertise<sensor_msgs::Image>(
				     node_name_ + "/cam_0/image_raw", 5);
		cam_0_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
					  node_name_ + "/cam_0/camera_info", 5);
	}

	// initialize imu msg publisher
	imu0_publisher_ = nh_.advertise<sensor_msgs::Imu>("cam_0/imu", 20);
	imu1_publisher_ = nh_.advertise<sensor_msgs::Imu>("cam_1/imu", 20);
	// wait on heart beat
	std::cout << "Waiting on device.";
	fflush(stdout);
	mavlink_message_t message;
	if(serial_port_open_ && nh_.ok()){
		int res = sp_.read_message(message);
		if(res == -1){
			serial_port_open_ = false;
			return;
		}

	mavlink_heartbeat_t heartbeat;

	while (heartbeat.type !=9){ //check for system type 9 heartbeat

		if(heartbeat.type!=9){
			printf(".");
			fflush(stdout);
			usleep(50000);
		}
		if(message.msgid ==0 ){
			mavlink_msg_heartbeat_decode(&message, &heartbeat);
			if(heartbeat.type ==9){
				std::cout << std::endl;
				ROS_INFO("Got heartbeat from camera");
			}
		}
		if(serial_port_open_ && nh_.ok()){
			int res = sp_.read_message(message);
			if(res == -1){
				serial_port_open_ = false;
				return;
			}
		}
	}

	//std::cout  << "Device connected" << std::endl;
	//setCalibration(camera_params_);
	// set flag for completed initializiation
	device_initialized_ = true;
	}
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::startDevice()
{
	if (device_initialized_) {

		setCalibration(camera_params_);

		dynamic_reconfigure_.setCallback(std::bind(&uvcROSDriver::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));

		// open uvc stream
		uvc_error_t res = initAndOpenUvc();
		// start stream
		past_ = ros::Time::now();
		res = uvc_start_streaming(devh_, &ctrl_, &callback, this, 0);
			//mavlink_param_value_t param;
			//bool wait=1;
			//std::string str = "CAMERA_ENABLE";
		setParam("CAMERA_ENABLE",float(camera_config_));
			//printf("Wait on camera \n");
		//usleep(200000);

		/*while(!uvc_cb_flag_ && ros::ok()){
				/*mavlink_message_t message;
				int res = sp_.read_message(message);
				if(res != 0){
					if(message.msgid==22){
						//printf("got message id 22...\n");
						mavlink_msg_param_value_decode(&message, &param);
						if(str.compare(param.param_id)==0 && param.param_value==float(camera_config_)){
							wait = 0;
							printf("camera started\n");
						}
						else{
							setParam("CAMERA_ENABLE",float(camera_config_));
							std::cout << "." ;
						}
					}

				}*/
				//std::cout << "." ;
			/*setParam("CAMERA_ENABLE",float(camera_config_));
			usleep(200000);
		}*/

			//printf(" done\n");
		printf("Waiting on stream");
		while (!uvc_cb_flag_ && ros::ok()) {
			printf(".");
			fflush(stdout);
			//uvc_stop_streaming(devh_);
			//res = uvc_start_streaming(devh_, &ctrl_, &callback, this, 0);
			
			if(setParam("CAMERA_ENABLE",float(camera_config_))==-1){
				ROS_ERROR("Device not initialized!");
				return;
			}
			usleep(200000);
			// std::cout << "res: " << res << std::endl;
		}

	} else {
		ROS_ERROR("Device not initialized!");
	}
}

////////////////////////////////////////////////////////////////////////////////

int uvcROSDriver::setParam(const std::string &name, float val)
{
	mavlink_message_t msg;
	char name_buf[16] = {};

	uint8_t local_sys = 1;
	uint8_t local_comp = 1;

	uint8_t target_sys = 99;
	uint8_t target_comp = 55;

	// SETCALIB
	strncpy(name_buf, name.c_str(), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
	mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys,
				   target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
	int ret = sp_.write_message(msg);

	if (ret <= 0) {
		printf("ret: %d\n", ret);
		return ret;
	}

	// another time, just so we maximize chances things actually go through
	strncpy(name_buf, name.c_str(), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
	mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys,
				   target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
	ret = sp_.write_message(msg);

	if (ret <= 0) {
		printf("ret: %d\n", ret);
		return ret;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::sendCameraParam(const int camera_number, const double fx,
				   const double fy, const Eigen::Vector2d &p0,
				   const float k1, const float k2,
				   const float r1, const float r2,
				   const Eigen::Matrix3d &H)
{
	std::string camera_name = "CAM" + std::to_string(camera_number);
	setParam("PARAM_CCX_" + camera_name, p0[0]);
	setParam("PARAM_CCY_" + camera_name, p0[1]);
	setParam("PARAM_FCX_" + camera_name, fx);
	setParam("PARAM_FCY_" + camera_name, fy);
	setParam("PARAM_KC1_" + camera_name, k1);
	setParam("PARAM_KC2_" + camera_name, k2);
	setParam("PARAM_P1_" + camera_name, r1);
	setParam("PARAM_P2_" + camera_name, r2);
	setParam("PARAM_H11_" + camera_name, H(0, 0));
	setParam("PARAM_H12_" + camera_name, H(0, 1));
	setParam("PARAM_H13_" + camera_name, H(0, 2));
	setParam("PARAM_H21_" + camera_name, H(1, 0));
	setParam("PARAM_H22_" + camera_name, H(1, 1));
	setParam("PARAM_H23_" + camera_name, H(1, 2));
	setParam("PARAM_H31_" + camera_name, H(2, 0));
	setParam("PARAM_H32_" + camera_name, H(2, 1));
	setParam("PARAM_H33_" + camera_name, H(2, 2));
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::setCalibration(CameraParameters camParams)
{
	// uvc_ros_driver::FPGACalibration cams[n_cameras_];
	std::vector<uvc_ros_driver::FPGACalibration> cams;
	int stereo_number = 0;

	if (camParams.isValid) {
		// TODO: find better way for this, general case not only stereo between
		// cam0->cam1, cam2->cam3
		for (int cam = 0; cam < n_cameras_; cam++) {
			uvc_ros_driver::FPGACalibration camera;
			camera.projection_model_.type_ =
				uvc_ros_driver::ProjectionModelTypes::PINHOLE;
			camera.projection_model_.focal_length_u_ = camParams.FocalLength[cam][0];
			camera.projection_model_.focal_length_v_ = camParams.FocalLength[cam][1];
			camera.projection_model_.principal_point_u_ =
				camParams.PrincipalPoint[cam][0];
			camera.projection_model_.principal_point_v_ =
				camParams.PrincipalPoint[cam][1];
			camera.projection_model_.k1_ = camParams.DistortionCoeffs[cam][0];
			camera.projection_model_.k2_ = camParams.DistortionCoeffs[cam][1];
			camera.projection_model_.r1_ = camParams.DistortionCoeffs[cam][2];
			camera.projection_model_.r2_ = camParams.DistortionCoeffs[cam][3];

			if (cam % 2 != 0) {
				camera.projection_model_.R_[0] =
					camParams.StereoTransformationMatrix[stereo_number][0][0];
				camera.projection_model_.R_[1] =
					camParams.StereoTransformationMatrix[stereo_number][0][1];
				camera.projection_model_.R_[2] =
					camParams.StereoTransformationMatrix[stereo_number][0][2];
				camera.projection_model_.R_[3] =
					camParams.StereoTransformationMatrix[stereo_number][1][0];
				camera.projection_model_.R_[4] =
					camParams.StereoTransformationMatrix[stereo_number][1][1];
				camera.projection_model_.R_[5] =
					camParams.StereoTransformationMatrix[stereo_number][1][2];
				camera.projection_model_.R_[6] =
					camParams.StereoTransformationMatrix[stereo_number][2][0];
				camera.projection_model_.R_[7] =
					camParams.StereoTransformationMatrix[stereo_number][2][1];
				camera.projection_model_.R_[8] =
					camParams.StereoTransformationMatrix[stereo_number][2][2];
				camera.projection_model_.t_[0] =
					camParams.StereoTransformationMatrix[stereo_number][0][3];
				camera.projection_model_.t_[1] =
					camParams.StereoTransformationMatrix[stereo_number][1][3];
				camera.projection_model_.t_[2] =
					camParams.StereoTransformationMatrix[stereo_number][2][3];
				stereo_number++;

			} else {
				camera.projection_model_.R_[0] = 1.0f;
				camera.projection_model_.R_[1] = 0.0f;
				camera.projection_model_.R_[2] = 0.0f;
				camera.projection_model_.R_[3] = 0.0f;
				camera.projection_model_.R_[4] = 1.0f;
				camera.projection_model_.R_[5] = 0.0f;
				camera.projection_model_.R_[6] = 0.0f;
				camera.projection_model_.R_[7] = 0.0f;
				camera.projection_model_.R_[8] = 1.0f;
				camera.projection_model_.t_[0] = 0.0f;
				camera.projection_model_.t_[1] = 0.0f;
				camera.projection_model_.t_[2] = 0.0f;
			}

			cams.push_back(camera);
		}

		// initialize vectors
		f_.resize(n_cameras_);
		p_.resize(n_cameras_);
		H_.resize(n_cameras_);
		// pointer at camera info
		sensor_msgs::CameraInfo *ci;
		size_t homography_size;

		// TODO: reimplment this part for multiple stereo base line based systems
		if (set_calibration_) {
			std::vector<std::pair<int, int> >::iterator it_homography =
				std::max_element(homography_mapping_.begin(), homography_mapping_.end(),
						 myPairMax);
			/*if ((*it_homography).second > n_cameras_) {
				homography_size = std::distance(homography_mapping_.begin(), it_homography);

			} else {
				homography_size = homography_mapping_.size();
			}*/
			//set homography number to number of camera pairs for now
			homography_size = n_cameras_/2;

			for (size_t i = 0; i < homography_size; i++) {
				// temp structures
				Eigen::Matrix3d H0;
				Eigen::Matrix3d H1;
				double f_new;
				Eigen::Vector2d p0_new, p1_new;

				std::pair<int, int> indx = homography_mapping_[i];

				StereoHomography h(cams[indx.first], cams[indx.second]);
				h.getHomography(H0, H1, f_new, p0_new, p1_new);

				f_[indx.first] = f_new;
				f_[indx.second] = f_new;
				p_[indx.first] = p0_new;
				p_[indx.second] = p1_new;
				// TODO check if matrix is copied or only pointer!!
				H_[indx.first] = H0;
				H_[indx.second] = H1;
			}

			// Set all parameters here
			for (int i = 0; i < n_cameras_; i++) {
				sendCameraParam(i, f_[i], f_[i], p_[i], cams[i].projection_model_.k1_,
						cams[i].projection_model_.k2_,
						cams[i].projection_model_.r1_,
						cams[i].projection_model_.r2_, H_[i]);
				selectCameraInfo(i, &ci);
				setCameraInfoIntrinsics(*ci, f_[i], f_[i], p_[i](0), p_[i](1));
				setCameraInfoDistortionMdl(
					*ci, uvc_ros_driver::ProjectionModelTypes::PINHOLE);
				setCameraInfoDistortionParams(*ci, 0, 0, 0, 0, 0);
			}

		} else {
			for (int i = 0; i < n_cameras_; i++) {
				selectCameraInfo(i, &ci);
				setCameraInfoIntrinsics(
					*ci, camParams.FocalLength[i][0], camParams.FocalLength[i][1],
					camParams.PrincipalPoint[i][0], camParams.PrincipalPoint[i][1]);
				setCameraInfoDistortionMdl(
					*ci, uvc_ros_driver::ProjectionModelTypes::PINHOLE);
				setCameraInfoDistortionParams(
					*ci, cams[i].projection_model_.k1_, cams[i].projection_model_.k2_,
					cams[i].projection_model_.r1_, cams[i].projection_model_.r2_, 0);
			}
		}


		// TODO: implement with class variables
		// SGM stereo penalty values p1: discontinuits, p2:
		setParam("STEREO_P1_CAM1", 10.0f);
		setParam("STEREO_P2_CAM1", 250.0f);
		// disparity L->R occlusion in px
		setParam("STEREO_LR_CAM1", 3.0f);
		// threshold 0-255 valid disparity
		setParam("STEREO_TH_CAM1", 140.0f);
		setParam("STEREO_FP_CAM1", 1.0f);
		setParam("STEREO_CE_CAM1", 0.0f);
		setParam("STEREO_RE_CAM1", 0.0f);
		setParam("STEREO_OF_CAM1", 0.0f);

		//setParam("COST_SHIFT", 2.0f);

		setParam("CAMERA_AUTOEXP", 1.0f);
		setParam("CAMERA_MAX_E", 60.0f);
		// setParam("CAMERA_EXP", 10.0f);
		// setParam("CAMERA_AUTOG",0.0f);
		// setParam("CAMERA_GAIN",63.0f);

		setParam("STEREO_MP_01", 0.0f);
		setParam("STEREO_BAYER_D", 0.0f);
		setParam("IMU_ENABLE", 2.0f);

		setParam("STEREO_P1_CAM3", 10.0f);
		setParam("STEREO_P2_CAM3", 250.0f);
		setParam("STEREO_LR_CAM3", 3.0f);
		setParam("STEREO_TH_CAM3", 140.0f);
		setParam("STEREO_FP_CAM3", 0.0f);
		setParam("STEREO_CE_CAM3", 0.0f);
		setParam("STEREO_RE_CAM3", 0.0f);
		setParam("STEREO_OF_CAM3", 0.0f);

		setParam("STEREO_P1_CAM5", 16.0f);
		setParam("STEREO_P2_CAM5", 240.0f);
		setParam("STEREO_LR_CAM5", 4.0f);
		setParam("STEREO_TH_CAM5", 120.0f);
		setParam("STEREO_FP_CAM5", 0.0f);
		setParam("STEREO_CE_CAM5", 0.0f);
		setParam("STEREO_RE_CAM5", 0.0f);
		setParam("STEREO_OF_CAM5", 0.0f);

		setParam("STEREO_P1_CAM7", 16.0f);
		setParam("STEREO_P2_CAM7", 240.0f);
		setParam("STEREO_LR_CAM7", 4.0f);
		setParam("STEREO_TH_CAM7", 120.0f);
		setParam("STEREO_FP_CAM7", 0.0f);
		setParam("STEREO_CE_CAM7", 0.0f);
		setParam("STEREO_RE_CAM7", 0.0f);
		setParam("STEREO_OF_CAM7", 0.0f);

		setParam("STEREO_P1_CAM9", 16.0f);
		setParam("STEREO_P2_CAM9", 240.0f);
		setParam("STEREO_LR_CAM9", 4.0f);
		setParam("STEREO_TH_CAM9", 120.0f);
		setParam("STEREO_FP_CAM9", 0.0f);
		setParam("STEREO_CE_CAM9", 0.0f);
		setParam("STEREO_RE_CAM9", 0.0f);
		setParam("STEREO_OF_CAM9", 0.0f);

		setParam("CALIB_GAIN", 4300.0f);

		setParam("CAMERA_H_FLIP", float(flip_));

		if(flip_){
			setParam("IM_H_FLIP_CAM0", 0.0f);
			setParam("IM_V_FLIP_CAM0", 0.0f);
			setParam("IM_H_FLIP_CAM2", 0.0f);
			setParam("IM_V_FLIP_CAM2", 0.0f);
			setParam("IM_H_FLIP_CAM4", 0.0f);
			setParam("IM_V_FLIP_CAM4", 0.0f);
			setParam("IM_H_FLIP_CAM6", 0.0f);
			setParam("IM_V_FLIP_CAM6", 0.0f);
			setParam("IM_H_FLIP_CAM8", 0.0f);
			setParam("IM_V_FLIP_CAM8", 0.0f);
			setParam("IM_H_FLIP_CAM1", 1.0f);
			setParam("IM_V_FLIP_CAM1", 1.0f);
			setParam("IM_H_FLIP_CAM3", 1.0f);
			setParam("IM_V_FLIP_CAM3", 1.0f);
			setParam("IM_H_FLIP_CAM5", 1.0f);
			setParam("IM_V_FLIP_CAM5", 1.0f);
			setParam("IM_H_FLIP_CAM7", 1.0f);
			setParam("IM_V_FLIP_CAM7", 1.0f);
			setParam("IM_H_FLIP_CAM9", 1.0f);
			setParam("IM_V_FLIP_CAM9", 1.0f);
		}
		else{
			setParam("IM_H_FLIP_CAM0", 1.0f);
			setParam("IM_V_FLIP_CAM0", 1.0f);
			setParam("IM_H_FLIP_CAM2", 1.0f);
			setParam("IM_V_FLIP_CAM2", 1.0f);
			setParam("IM_H_FLIP_CAM4", 1.0f);
			setParam("IM_V_FLIP_CAM4", 1.0f);
			setParam("IM_H_FLIP_CAM6", 1.0f);
			setParam("IM_V_FLIP_CAM6", 1.0f);
			setParam("IM_H_FLIP_CAM8", 1.0f);
			setParam("IM_V_FLIP_CAM8", 1.0f);
			setParam("IM_H_FLIP_CAM1", 0.0f);
			setParam("IM_V_FLIP_CAM1", 0.0f);
			setParam("IM_H_FLIP_CAM3", 0.0f);
			setParam("IM_V_FLIP_CAM3", 0.0f);
			setParam("IM_H_FLIP_CAM5", 0.0f);
			setParam("IM_V_FLIP_CAM5", 0.0f);
			setParam("IM_H_FLIP_CAM7", 0.0f);
			setParam("IM_V_FLIP_CAM7", 0.0f);
			setParam("IM_H_FLIP_CAM9", 0.0f);
			setParam("IM_V_FLIP_CAM9", 0.0f);
		}

	}

	/*if (set_calibration_) {
		setParam("RESETCALIB", 0.0f);

	} else {
		setParam("RESETCALIB", 1.0f);
	}*/


	setParam("SETCALIB", float(set_calibration_));

	// std::cout << "Configuring cameras..." << std::endl;
	setParam("RESETMT9V034", 1.0f);
	setParam("RESETICM20608",1.0f);
	// sleep(5);  // needed, fpga reconfigure cameras and restart time
	// std::cout << "Configuration completed." << std::endl;
	// last 4 bits activate the 4 camera pairs 0x01 = pair 1 only, 0x0F all 4
	// pairs
	//setParam("CAMERA_ENABLE", float(camera_config_));

}

////////////////////////////////////////////////////////////////////////////////
// NOTE: return error really necessary?
uvc_error_t uvcROSDriver::initAndOpenUvc()
{
	uvc_error_t res;
	/* Initialize a UVC service context. Libuvc will set up its own libusb
	* context. Replace NULL with a libusb_context pointer to run libuvc
	* from an existing libusb context. */
	res = uvc_init(&ctx_, NULL);

	if (res < 0) {
		uvc_perror(res, "uvc_init");
		ROS_ERROR("Unable to initialize uvc service context");
		return res;
	}

	/* Locates the first attached UVC device, stores in dev */
	/* filter devices: vendor_id, product_id, "serial_num" */
	res = uvc_find_device(ctx_, &dev_, 0x04b4, 0, NULL);

	if (res < 0) {
		uvc_perror(res, "uvc_find_device"); /* no devices found */
		ROS_ERROR("No devices found");
		return res;
	}

	ROS_INFO("Device found");

	/* Try to open the device: requires exclusive access */
	res = uvc_open(dev_, &devh_);

	if (res < 0) {
		uvc_perror(res, "uvc_open"); /* unable to open device */
		ROS_ERROR("Unable to open the device");
		return res;
	}

	// ?????????
	// uvc_device_descriptor_t *desc;
	// uvc_get_device_descriptor(dev_, &desc);
	// uvc_free_device_descriptor(desc);

	/* Try to negotiate a 640x480 30 fps YUYV stream profile */
	res = uvc_get_stream_ctrl_format_size(
		      devh_, &ctrl_,              /* result stored in ctrl */
		      UVC_FRAME_FORMAT_YUYV,      /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
		      raw_width_, raw_height_, 30 /* width, height, fps */
	      );

	if (res < 0) {
		/* device doesn't provide a matching stream */
		uvc_perror(res, "get_mode");
		ROS_ERROR("Device doesn't provide a matching stream");
		return res;
	}

	return res;
}

////////////////////////////////////////////////////////////////////////////////

int16_t uvcROSDriver::ShortSwap(int16_t s)
{
	unsigned char b1, b2;
	b1 = s & 255;
	b2 = (s >> 8) & 255;
	return (b1 << 8) + b2;
}

////////////////////////////////////////////////////////////////////////////////

inline void uvcROSDriver::selectCameraInfo(int camera,
		sensor_msgs::CameraInfo **ci)
{
	switch (camera) {
	case 0:
		*ci = &info_cam_0_;
		break;

	case 1:
		*ci = &info_cam_1_;
		break;

	case 2:
		*ci = &info_cam_2_;
		break;

	case 3:
		*ci = &info_cam_3_;
		break;

	case 4:
		*ci = &info_cam_4_;
		break;

	case 5:
		*ci = &info_cam_5_;
		break;

	case 6:
		*ci = &info_cam_6_;
		break;

	case 7:
		*ci = &info_cam_7_;
		break;

	case 8:
		*ci = &info_cam_8_;
		break;

	case 9:
		*ci = &info_cam_9_;
		break;
	}
}

void uvcROSDriver::dynamicReconfigureCallback(
    uvc_ros_driver::UvcDriverConfig &config, uint32_t level)
{
    setParam("CAMERA_AUTOEXP", static_cast<float>(config.CAMERA_AUTOEXP));
    setParam("CAMERA_EXP", static_cast<float>(config.CAMERA_EXP));
    setParam("CAMERA_AUTOG", static_cast<float>(config.CAMERA_AUTOG));
    setParam("CAMERA_GAIN", static_cast<float>(config.CAMERA_GAIN));

    setParam("ADIS_IMU", static_cast<float>(config.ADIS_IMU));

    setParam("UPDATEMT9V034", 1.0f);
}

////////////////////////////////////////////////////////////////////////////////

inline void uvcROSDriver::deinterleave(const uint8_t *mixed, uint8_t *array1,
				       uint8_t *array2, size_t mixedLength,
				       size_t imageWidth, size_t imageHeight)
{
	int i = 0;
	int c = 0;
#if defined __ARM_NEON__
	size_t vectors = mixedLength / 32;
	size_t divider = (imageWidth + 16) * 2 / 32;

	while (vectors-- > 0) {
		const uint8x16_t src0 = vld1q_u8(mixed);
		const uint8x16_t src1 = vld1q_u8(mixed + 16);
		const uint8x16x2_t dst = vuzpq_u8(src0, src1);

		if (vectors % divider != 0) {
			vst1q_u8(array1, dst.val[0]);
			vst1q_u8(array2, dst.val[1]);
			array1 += 16;
			array2 += 16;
			c += 16;
		}

		i += 16;
		mixed += 32;
	}

#endif

	while (c < imageWidth * imageHeight) {
		array1[c] = mixed[2 * i];
		array2[c] = mixed[2 * i + 1];
		i++;
		c++;

		if ((c % imageWidth) == 0) {
			i += 16;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

/* This callback function runs once per frame. Use it to perform any
  quick processing you need, or have it put the frame into your application's
  input queue.If this function takes too long, you'll start losing frames. */
void uvcROSDriver::uvc_cb(uvc_frame_t *frame)
{
	// check if evrytstartedhing ok
	if (!ros::ok()) {
		return;
	}

	if(!uvc_cb_flag_){
		std::cout << std::endl;
		ROS_INFO("Stream started");
	}
	// flag
	uvc_cb_flag_ = true;

	ait_ros_messages::VioSensorMsg msg_vio;
	sensor_msgs::Imu msg_imu;

	unsigned frame_size = frame->height * frame->width * 2;

	// read the IMU data
	uint16_t cam_id = 0;
	uint16_t imu_id = 0;
	static uint16_t count_prev = 0;

	// read out micro second timestamp of 1st line
	uint16_t timestamp_upper =
		((static_cast<uint16_t *>(frame->data)[int((1) * frame->width - 10)]) >>
		 8) +
		(((static_cast<uint16_t *>(frame->data)[int((1) * frame->width - 10)]) &
		  255)
		 << 8);
	uint16_t timestamp_lower =
		((static_cast<uint16_t *>(frame->data)[int((1) * frame->width - 9)]) >>
		 8) +
		(((static_cast<uint16_t *>(frame->data)[int((1) * frame->width - 9)]) &
		  255)
		 << 8);
	uint32_t timestamp =
		(((uint32_t)(timestamp_upper)) << 16) + (uint32_t)(timestamp_lower);

    // Static vars are initialized only in the first run. Calculate time offset between current time (ros::Time::now())
    // of host and substract current timestamp of device, as this timestamp depends on the powered on time of the device
	// static ros::Duration time_offset_frame(0.041);
	static ros::Duration time_offset_frame(0.0);
	static ros::Time fpga_frame_time = ros::Time::now() - time_offset_frame - ros::Duration(double(timestamp/k_ms_to_sec)); //subtract first timestamp
	static ros::Time fpga_line_time = ros::Time::now() - ros::Duration(double(timestamp/k_ms_to_sec));
	ros::Duration fpga_time_add(0.0);

	// wrap around is automatically handled by underflow of uint16_t values
	fpga_time_add = ros::Duration((double(timestamp - time_wrapper_check_frame_)) /k_ms_to_sec);
	time_wrapper_check_frame_ = timestamp;

	// frame time is timestamp of 1st line + offset_start - offset_frame from
	// exposure to readout of 1st line
	fpga_frame_time = fpga_frame_time + fpga_time_add;

	unsigned int imu_msg_counter_in_frame = 0;
	ros::Time timestamp_second_imu_msg_in_frame;

	for (unsigned i = 0; i < frame->height; i++) {

		uint16_t count = ShortSwap(static_cast<uint16_t *>(
						   frame->data)[int((i + 1) * frame->width - 8 + 0)]);

		// detect cam_id in first row

		//bit 15-12 cam_id, 11-8 imu_id, 7-0 counter 

		if (i == 0) {
			cam_id = (count & 0xF000) >> 12;
		}

		imu_id = (count & 0x0F00) >> 8;

		count = count & 0x00FF;

		double temperature = double(ShortSwap(static_cast<int16_t *>(
				frame->data)[int((i + 1) * frame->width - 8 + 1)]));

		double acc_x = double(ShortSwap(static_cast<int16_t *>(
							frame->data)[int((i + 1) * frame->width - 8 + 2)])) /
			       (acc_scale_factor / 9.81);
		double acc_y = double(ShortSwap(static_cast<int16_t *>(
							frame->data)[int((i + 1) * frame->width - 8 + 3)])) /
			       (acc_scale_factor / 9.81);
		double acc_z = double(ShortSwap(static_cast<int16_t *>(
							frame->data)[int((i + 1) * frame->width - 8 + 4)])) /
			       (acc_scale_factor / 9.81);

		double gyr_x =
			double(ShortSwap(static_cast<int16_t *>(
						 frame->data)[int((i + 1) * frame->width - 8 + 5)]) /
			       (gyr_scale_factor / deg2rad));
		double gyr_y =
			double(ShortSwap(static_cast<int16_t *>(
						 frame->data)[int((i + 1) * frame->width - 8 + 6)]) /
			       (gyr_scale_factor / deg2rad));
		double gyr_z =
			double(ShortSwap(static_cast<int16_t *>(
						 frame->data)[int((i + 1) * frame->width - 8 + 7)]) /
			       (gyr_scale_factor / deg2rad));

		// read out micro second timestamp of corresponding line
		timestamp_upper = ((static_cast<uint16_t *>(
					    frame->data)[int((i + 1) * frame->width - 10)]) >>
				   8) +
				  (((static_cast<uint16_t *>(
					     frame->data)[int((i + 1) * frame->width - 10)]) &
				    255)
				   << 8);
		timestamp_lower = ((static_cast<uint16_t *>(
					    frame->data)[int((i + 1) * frame->width - 9)]) >>
				   8) +
				  (((static_cast<uint16_t *>(
				      frame->data)[int((i + 1) * frame->width - 9)]) &
				      255)
				      << 8);
		timestamp =
		    (((uint32_t)(timestamp_upper)) << 16) + (uint32_t)(timestamp_lower);

		// wrap around is automatically handled by underflow of uint16_t values
		fpga_time_add =
		    ros::Duration(double(timestamp - time_wrapper_check_line_) / k_ms_to_sec);
		time_wrapper_check_line_ = timestamp;

		// line time is timestamp of current line + offset_start
		fpga_line_time = fpga_line_time + fpga_time_add;

		if(first_imu_received_flag_ == false) {
		  first_imu_received_flag_ = true;
		  timestamp_prev_imu_msg_ = fpga_line_time;
		  imu_dt_ = ros::Duration(0.0);
		}



		if (!(count == count_prev)) {

			ros::Time imu_timestamp;
			if(imu_msg_counter_in_frame > 0)
			{
				imu_timestamp = fpga_line_time;
			} else {
				// for the first imu message in an image take the previous imu msg timestamp and add imu sampling time.
				imu_timestamp = timestamp_prev_imu_msg_ + imu_dt_;

				// TODO(burrimi): FPGA sometimes drops an IMU if it arrives between images. Try to catch and correct timestamp.
				// This sometimes triggers a false positive, commenting out for now.
				// const ros::Duration time_diff(fpga_line_time - timestamp_prev_imu_msg_);
				// if(time_diff.toSec() > 0.002124) { // Magic constant is overconservative, might not catch all imu drops.
				//	//std::cout << "imu lost " << time_diff.toSec() << std::endl;
				//	imu_timestamp += imu_dt_;
				//}
			}

			timestamp_prev_imu_msg_ = imu_timestamp;
			if(imu_msg_counter_in_frame == 1) {
				timestamp_second_imu_msg_in_frame = imu_timestamp;
			}

			if (flip_) {
				msg_imu.linear_acceleration.x = acc_y;
				msg_imu.linear_acceleration.y = acc_x;
				msg_imu.linear_acceleration.z = -acc_z;

				msg_imu.angular_velocity.x = gyr_y;
				msg_imu.angular_velocity.y = gyr_x;
				msg_imu.angular_velocity.z = -gyr_z;

			} else {
				// TODO: check if correct
				msg_imu.linear_acceleration.x = -acc_y;
				msg_imu.linear_acceleration.y = -acc_x;
				msg_imu.linear_acceleration.z = -acc_z;

				msg_imu.angular_velocity.x = -gyr_y;
				msg_imu.angular_velocity.y = -gyr_x;
				msg_imu.angular_velocity.z = -gyr_z;
			}

			msg_imu.header.stamp = imu_timestamp;

			msg_vio.imu.push_back(msg_imu);

			if (imu_id == 1){
				imu1_publisher_.publish(msg_imu);
			} else {
				imu0_publisher_.publish(msg_imu);
			}

			++imu_msg_counter_in_frame;

			count_prev = count;
		}
	}

	if(imu_msg_counter_in_frame > 3) {
		const ros::Duration diff_timestamps_imu = timestamp_prev_imu_msg_ - timestamp_second_imu_msg_in_frame;
		imu_dt_.fromSec(diff_timestamps_imu.toSec() / (imu_msg_counter_in_frame - 2));
	}
	//std::cout << "total: " << imu_dt_ << " n: " << imu_msg_counter_in_frame << std::endl;

	ros::Duration elapsed = fpga_frame_time - past_;
	past_ = fpga_frame_time;

	ROS_DEBUG("camera id: %d   ", cam_id);
	ROS_DEBUG("Current time: %f", ros::Time::now().toSec());
	ROS_DEBUG("Message Timestamp: %f   ", fpga_frame_time.toSec());
	ROS_DEBUG("Device Timestamp: %f   ", double(timestamp/k_ms_to_sec));
	ROS_DEBUG("framerate: %f   ", 1.0 / elapsed.toSec());
	ROS_DEBUG("%lu imu messages", msg_vio.imu.size());
	ROS_DEBUG("imu id: %d ", imu_id);

	// temp container for the 2 images
	uint8_t left[(frame_size - 16 * 2 * frame->height) / 2];
	uint8_t right[(frame_size - 16 * 2 * frame->height) / 2];
	// read the image data and separate the 2 images
	deinterleave(static_cast<unsigned char *>(frame->data), left, right,
		     (size_t)frame_size, frame->width - 16, frame->height);

	sensor_msgs::fillImage(msg_vio.left_image,
			       sensor_msgs::image_encodings::BAYER_RGGB8,//MONO8,//
			       frame->height,      // height
			       frame->width - 16,  // width
			       frame->width - 16,  // stepSize
			       left);

	//msg_vio.left_image.header.stamp = fpga_frame_time;

	sensor_msgs::fillImage(msg_vio.right_image,
			       sensor_msgs::image_encodings::MONO8,//BAYER_RGGB8,//
			       frame->height,      // height
			       frame->width - 16,  // width
			       frame->width - 16,  // stepSize
			       right);

	//msg_vio.right_image.header.stamp = fpga_frame_time;


	// publish data
	if (cam_id == 0) {  // select_cam = 0 + 1
		// set timestamp of all frames when cameras 8+9 are disabled
		if(n_cameras_ < 9){
			frame_time_ = fpga_frame_time;
			frameCounter_++;
		}
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images and on msg_vio
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.frame_id = "cam_0_optical_frame";
		msg_vio.right_image.header.frame_id = "cam_1_optical_frame";

		if (frameCounter_ % modulo_ == 0) {
			// publish images and camera info
			cam_0_pub_.publish(msg_vio.left_image);
			cam_1_pub_.publish(msg_vio.right_image);

			// set camera info header
			setCameraInfoHeader(info_cam_0_, width_, height_, frame_time_,
					    "cam_0_optical_frame");
			setCameraInfoHeader(info_cam_1_, width_, height_, frame_time_,
					    "cam_1_optical_frame");
			// publish camera info
			cam_0_info_pub_.publish(info_cam_0_);
			cam_1_info_pub_.publish(info_cam_1_);
		}
	}

	if (cam_id == 8 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		//check if camera0/camera1 raw output is disabled and set timestamp 
		//when cameras 8+9 are disabled
		if(n_cameras_ < 9 && (camera_config_ & 0x001) ==0){
			frame_time_ = fpga_frame_time;
			frameCounter_++;
		}
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_0_corrected_frame";
		msg_vio.right_image.header.frame_id = "cam_0_disparity_frame";

		// publish images
		cam_0c_pub_.publish(msg_vio.left_image);
		cam_0d_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_2_, width_, height_, frame_time_,
				    "cam_2_optical_frame");
		setCameraInfoHeader(info_cam_3_, width_, height_, frame_time_,
				    "cam_3_optical_frame");
		// publish camera info
		cam_0c_info_pub_.publish(info_cam_0_);
		cam_0d_info_pub_.publish(info_cam_1_);
	}

	if (cam_id == 1 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_2_optical_frame";
		msg_vio.right_image.header.frame_id = "cam_3_optical_frame";

		// publish images
		cam_2_pub_.publish(msg_vio.left_image);
		cam_3_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_2_, width_, height_, frame_time_,
				    "cam_2_optical_frame");
		setCameraInfoHeader(info_cam_3_, width_, height_, frame_time_,
				    "cam_3_optical_frame");
		// publish camera info
		cam_2_info_pub_.publish(info_cam_2_);
		cam_3_info_pub_.publish(info_cam_3_);
	}

	if (cam_id == 9 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_2_corrected_frame";
		msg_vio.right_image.header.frame_id = "cam_2_disparity_frame";

		// publish images
		cam_2c_pub_.publish(msg_vio.left_image);
		cam_2d_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_2_, width_, height_, frame_time_,
				    "cam_2_optical_frame");
		setCameraInfoHeader(info_cam_3_, width_, height_, frame_time_,
				    "cam_3_optical_frame");
		// publish camera info
		cam_2c_info_pub_.publish(info_cam_2_);
		cam_2d_info_pub_.publish(info_cam_3_);
	}

	if (cam_id == 2 && frameCounter_ % modulo_ == 0) {  // select_cam = 4 + 5
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_4_optical_frame";
		msg_vio.right_image.header.frame_id = "cam_5_optical_frame";

		// publish images
		cam_4_pub_.publish(msg_vio.left_image);
		cam_5_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_4_, width_, height_, frame_time_,
				    "cam_4_optical_frame");
		setCameraInfoHeader(info_cam_5_, width_, height_, frame_time_,
				    "cam_5_optical_frame");
		// publish camera info
		cam_4_info_pub_.publish(info_cam_4_);
		cam_5_info_pub_.publish(info_cam_5_);
	}

	if (cam_id == 10 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_4_corrected_frame";
		msg_vio.right_image.header.frame_id = "cam_4_disparity_frame";

		// publish images
		cam_4c_pub_.publish(msg_vio.left_image);
		cam_4d_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_4_, width_, height_, frame_time_,
				    "cam_4_optical_frame");
		setCameraInfoHeader(info_cam_5_, width_, height_, frame_time_,
				    "cam_4_optical_frame");
		// publish camera info
		cam_4c_info_pub_.publish(info_cam_2_);
		cam_4d_info_pub_.publish(info_cam_3_);
	}

	if (cam_id == 3 && frameCounter_ % modulo_ == 0) {  // select_cam = 6 + 7
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_6_optical_frame";
		msg_vio.right_image.header.frame_id = "cam_7_optical_frame";

		// publish images
		cam_6_pub_.publish(msg_vio.left_image);
		cam_7_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_6_, width_, height_, frame_time_,
				    "cam_6_optical_frame");
		setCameraInfoHeader(info_cam_7_, width_, height_, frame_time_,
				    "cam_7_optical_frame");
		// publish camera info
		cam_6_info_pub_.publish(info_cam_6_);
		cam_7_info_pub_.publish(info_cam_7_);
	}

	if (cam_id == 11 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_6_corrected_frame";
		msg_vio.right_image.header.frame_id = "cam_6_disparity_frame";

		// publish images
		cam_6c_pub_.publish(msg_vio.left_image);
		cam_6d_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_2_, width_, height_, frame_time_,
				    "cam_2_optical_frame");
		setCameraInfoHeader(info_cam_3_, width_, height_, frame_time_,
				    "cam_3_optical_frame");
		// publish camera info
		cam_6c_info_pub_.publish(info_cam_2_);
		cam_6d_info_pub_.publish(info_cam_3_);
	}

	if (cam_id == 4 && frameCounter_ % modulo_ == 0) {  // select_cam = 8 + 9
		// set timestamp for all frames if cam 8 + 9 enabled here
		if(n_cameras_ >=9){
			frame_time_ = fpga_frame_time;
			frameCounter_++;
		}
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_8_optical_frame";
		msg_vio.right_image.header.frame_id = "cam_9_optical_frame";

		// publish images
		cam_8_pub_.publish(msg_vio.left_image);
		cam_9_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_8_, width_, height_, frame_time_,
				    "cam_8_optical_frame");
		setCameraInfoHeader(info_cam_9_, width_, height_, frame_time_,
				    "cam_9_optical_frame");
		// publish camera info
		cam_8_info_pub_.publish(info_cam_8_);
		cam_9_info_pub_.publish(info_cam_9_);
	}

	if (cam_id == 12 && frameCounter_ % modulo_ == 0) {  // select_cam = 2 + 3
		// FPGA can only send 2 images at time, but all of them are took at the same
		// time, so the image time stamp should be set to the first camera pair
		// set timestamp if cam 8 + 9 raw is disabled
		if(n_cameras_ >=9 ){//&& (camera_config_ & 0x010) ==0){
			frame_time_ = fpga_frame_time;
			frameCounter_++;
		}
		msg_vio.header.stamp = frame_time_;
		msg_vio.left_image.header.stamp = frame_time_;
		msg_vio.right_image.header.stamp = frame_time_;
		// set frame_id on images
		msg_vio.left_image.header.frame_id = "cam_8_corrected_frame";
		msg_vio.right_image.header.frame_id = "cam_8_disparity_frame";

		// publish images
		cam_8c_pub_.publish(msg_vio.left_image);
		cam_8d_pub_.publish(msg_vio.right_image);

		// set camera info header
		setCameraInfoHeader(info_cam_2_, width_, height_, frame_time_,
				    "cam_2_optical_frame");
		setCameraInfoHeader(info_cam_3_, width_, height_, frame_time_,
				    "cam_3_optical_frame");
		// publish camera info
		cam_8c_info_pub_.publish(info_cam_2_);
		cam_8d_info_pub_.publish(info_cam_3_);
	}

	ROS_DEBUG("Frame Time: %f \n", frame_time_.toSec());

	// realy needed?
	msg_vio.left_image.data.clear();
	msg_vio.right_image.data.clear();
	msg_vio.imu.clear();
}

} /* uvc */
