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
 * uvc_ros_driver.h
 *
 *  Created on: Jul 5, 2016
 *      Author: nicolas, christoph, simone
 *
 *  The code below is based on the example provided at
 *  https://int80k.com/libuvc/doc/
 */

#ifndef __UVC_ROS_DRIVER_H__
#define __UVC_ROS_DRIVER_H__

#if defined __ARM_NEON__
#include <arm_neon.h>
#endif

// local include
#include "serial_port.h"
#include "stereo_homography.h"
#include "fpga_calibration.h"
#include "calib_yaml_interface.h"
#include "camera_info_helper.h"

#include "libuvc/libuvc.h"

#include <ait_ros_messages/VioSensorMsg.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include <vector>
#include <utility>  // std::pair
#include <string>
#include <algorithm>

namespace uvc
{
class uvcROSDriver
{
private:
	bool device_initialized_ = false;
	bool enable_ait_vio_msg_ = false;
	bool flip_ = false;
	bool depth_map_ = false;
	bool set_calibration_ = false;
	bool uvc_cb_flag_ = false;
	bool first_imu_received_flag_ = false;

	int n_cameras_ = 2;
	int camera_config_ = 1;
	int raw_width_ = 768;
	int raw_height_ = 480;
	int width_ = raw_width_ - 16;
	int height_ = raw_height_;
	int frameCounter_ = 0;
	int modulo_ = 1;
	int calibration_mode_ = 0;

	ros::Duration imu_dt_ = ros::Duration(0.0);
  ros::Time timestamp_prev_imu_msg_;

	// TODO: add other camera parameters
	// float ....

	double acc_scale_factor = 16384.0;
	double gyr_scale_factor = 131.0;
	double deg2rad = 2 * M_PI / 360.0;

	// homography variables
	std::vector<std::pair<int, int>> homography_mapping_;
	std::vector<double> f_;
	std::vector<Eigen::Vector2d> p_;
	std::vector<Eigen::Matrix3d> H_;

	CameraParameters camera_params_;
	// serial port
	Serial_Port sp_;
	// uvc
	uvc_context_t *ctx_;
	uvc_device_t *dev_;
	uvc_device_handle_t *devh_;
	uvc_stream_ctrl_t ctrl_;
	// ros node handle
	ros::NodeHandle nh_;
	// node name
	std::string node_name_;
	// time
	ros::Time past_;
	ros::Time frame_time_;
	ros::Time start_offset_;
	uint32_t time_wrapper_check_frame_ = 0.0;
	uint32_t time_wrapper_check_line_ = 0.0;
	// image publishers
	ros::Publisher cam_0_pub_;
	ros::Publisher cam_0_info_pub_;
	ros::Publisher cam_0c_pub_;
	ros::Publisher cam_0c_info_pub_;
	ros::Publisher cam_0d_pub_;
	ros::Publisher cam_0d_info_pub_;
	ros::Publisher cam_1_pub_;
	ros::Publisher cam_1_info_pub_;
	ros::Publisher cam_2_pub_;
	ros::Publisher cam_2_info_pub_;
	ros::Publisher cam_2c_pub_;
	ros::Publisher cam_2c_info_pub_;
	ros::Publisher cam_2d_pub_;
	ros::Publisher cam_2d_info_pub_;
	ros::Publisher cam_3_pub_;
	ros::Publisher cam_3_info_pub_;
	ros::Publisher cam_4_pub_;
	ros::Publisher cam_4_info_pub_;
	ros::Publisher cam_4c_pub_;
	ros::Publisher cam_4c_info_pub_;
	ros::Publisher cam_4d_pub_;
	ros::Publisher cam_4d_info_pub_;
	ros::Publisher cam_5_pub_;
	ros::Publisher cam_5_info_pub_;
	ros::Publisher cam_6_pub_;
	ros::Publisher cam_6_info_pub_;
	ros::Publisher cam_6c_pub_;
	ros::Publisher cam_6c_info_pub_;
	ros::Publisher cam_6d_pub_;
	ros::Publisher cam_6d_info_pub_;
	ros::Publisher cam_7_pub_;
	ros::Publisher cam_7_info_pub_;
	ros::Publisher cam_8_pub_;
	ros::Publisher cam_8_info_pub_;
	ros::Publisher cam_8c_pub_;
	ros::Publisher cam_8c_info_pub_;
	ros::Publisher cam_8d_pub_;
	ros::Publisher cam_8d_info_pub_;
	ros::Publisher cam_9_pub_;
	ros::Publisher cam_9_info_pub_;
	// vio publishers
	ros::Publisher stereo_vio_1_pub_;
	ros::Publisher stereo_vio_2_pub_;
	ros::Publisher stereo_vio_3_pub_;
	ros::Publisher stereo_vio_4_pub_;
	ros::Publisher stereo_vio_5_pub_;
	// imu publisher
	ros::Publisher imu_publisher_;
	// camera info
	sensor_msgs::CameraInfo info_cam_0_;
	sensor_msgs::CameraInfo info_cam_1_;
	sensor_msgs::CameraInfo info_cam_2_;
	sensor_msgs::CameraInfo info_cam_3_;
	sensor_msgs::CameraInfo info_cam_4_;
	sensor_msgs::CameraInfo info_cam_5_;
	sensor_msgs::CameraInfo info_cam_6_;
	sensor_msgs::CameraInfo info_cam_7_;
	sensor_msgs::CameraInfo info_cam_8_;
	sensor_msgs::CameraInfo info_cam_9_;

	int16_t ShortSwap(int16_t s);
	uvc_error_t initAndOpenUvc();
	int setParam(const std::string &name, float val);
	void sendCameraParam(const int camera_number, const double fx,
			     const double fy, const Eigen::Vector2d &p0,
			     const float k1, const float k2, const float r1,
			     const float r2, const Eigen::Matrix3d &H);
	void setCalibration(CameraParameters camParams);
	inline void deinterleave(const uint8_t *mixed, uint8_t *array1,
				 uint8_t *array2, size_t mixedLength,
				 size_t imageWidth, size_t imageHeight);
	inline void selectCameraInfo(int camera, sensor_msgs::CameraInfo **ci);

public:
	uvcROSDriver(ros::NodeHandle nh)
		: nh_(nh), node_name_(ros::this_node::getName()) {};
	~uvcROSDriver();
	void uvc_cb(uvc_frame_t *frame);
	/**
	 * initialize device, set up topic publishers and compute homography for
	 * the
	 * cameras
	 */
	void initDevice();
	/**
	 * setup uvc stream
	 */
	void startDevice();
	// getter and setter for different internal variables
	bool getUseOfAITMsgs()
	{
		return enable_ait_vio_msg_;
	};
	void setUseOFAITMsgs(bool enable)
	{
		enable_ait_vio_msg_ = enable;
	};
	bool getFlip()
	{
		return flip_;
	};
	void setFlip(bool flip)
	{
		flip_ = flip;
	};
	bool getUseOfDepthMap()
	{
		return depth_map_;
	};
	void setUseOfDepthMap(bool depth_map)
	{
		depth_map_ = depth_map;
	};
	bool getCalibrationParam()
	{
		return set_calibration_;
	};
	void setCalibrationParam(bool calibration)
	{
		set_calibration_ = calibration;
	};
	int getNumberOfCameras()
	{
		return n_cameras_;
	};
	void setNumberOfCameras(int n_cameras)
	{
		n_cameras_ = n_cameras;

		switch (n_cameras) {
		case 8:
			camera_config_ = 0x1EF;
			break;

		case 6:
			camera_config_ = 0xE7;
			break;

		case 4:
			camera_config_ = 0x63;
			break;

		case 2:
		default:
			camera_config_ = 0x21;
			break;
		}
	};
	int getCameraConfig()
	{
		return camera_config_;
	};
	//	void setCameraConfig(int camera_config)
	//	{
	//		camera_config_ = camera_config;
	//	};
	CameraParameters getCameraParams()
	{
		return camera_params_;
	};
	void setCameraParams(const CameraParameters &camera_params)
	{
		camera_params_ = camera_params;
	};
	void getHomographyMapping(
		std::vector<std::pair<int, int>> &homography_mapping)
	{
		homography_mapping = homography_mapping_;
	};
	void setHomographyMapping(
		const std::vector<std::pair<int, int>> &homography_mapping)
	{
		homography_mapping_ = homography_mapping;
	};
	int getCalibrationMode()
	{
		return calibration_mode_;
	};
	void setCalibrationMode(int calibration_mode)
	{
		calibration_mode_ = calibration_mode;

		// update modulo_ variable also
		if (calibration_mode != 0) {
			modulo_ = 12;
		}
	};
};

} /* uvc */

#endif /* end of include guard: __UVC_ROS_DRIVER_H__ */
