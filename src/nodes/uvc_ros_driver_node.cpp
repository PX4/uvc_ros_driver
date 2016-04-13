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
 * uvc_ros_driver_node.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: nicolas
 *
 *  The code below is based on the example provided at https://int80k.com/libuvc/doc/
 */

#include <stdio.h>
#include <unistd.h>
#include <sstream>

#include "libuvc/libuvc.h"
#include <ros/ros.h>

#include "ait_ros_messages/VioSensorMsg.h"
#include "uvc_ros_driver/calibration.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include "serial_port.h"
#include "stereo_homography.h"
#include "fpga_calibration.h"

static const double acc_scale_factor = 16384.0;
static const double gyr_scale_factor = 131.0;
static const double deg2rad = 2 * M_PI / 360.0;

//static double acc_x_prev, acc_y_prev, acc_z_prev, gyr_x_prev, gyr_y_prev, gyr_z_prev;
static uint16_t count_prev;
static ros::Time last_time;
ait_ros_messages::VioSensorMsg msg;

// struct holding all data needed in the callback
struct UserData {
	ros::Publisher pub;
	bool hflip;
	bool serialconfig;
};

int16_t ShortSwap(int16_t s)
{
	unsigned char b1, b2;

	b1 = s & 255;
	b2 = (s >> 8) & 255;

	return (b1 << 8) + b2;

}

// This callback function is executed at a callback during start up.
// It is only used to signal that we got a callback
void startup_cb(uvc_frame_t *frame, void *user_ptr)
{
	bool *got_cb = (bool *) user_ptr;
	*got_cb = true;
}

// It helps to repeatedly start and stop the stream in order for the images to stream properly.
// So we do this until we get a first callback
bool repeatedStart(uvc_device_handle_t *devh, uvc_stream_ctrl_t ctrl)
{
	bool got_cb = false;

	int max_attempts = 100;
	int attempts = 0;

	ros::Rate r(100);
	ros::Duration dur(1); // sleep for 1 sec between retries

	while (!got_cb && attempts < max_attempts) {

		uvc_error_t res = uvc_start_streaming(devh, &ctrl, startup_cb, &got_cb, 0);

		if (res < 0) {
			uvc_perror(res, "start_streaming"); /* unable to start stream */
			return false;

		} else {
			ROS_DEBUG("[%d] Starting stream...\n", attempts + 1);

			ros::Time start_time = ros::Time::now();

			while (ros::Time::now() < start_time + dur) {
				ros::spinOnce();
				r.sleep();
			}

			/* End the stream. Blocks until last callback is serviced */
			uvc_stop_streaming(devh);

			if (got_cb) {
				ROS_INFO("Sucessfully started stream after %d attempts", attempts + 1);
				return true;
			}
		}

		attempts++;
	}

	// failed after max_attempts
	return false;
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void uvc_cb(uvc_frame_t *frame, void *user_ptr)
{

	UserData *user_data = (UserData *) user_ptr;

	//ait_ros_messages::VioSensorMsg msg;

	msg.header.stamp = ros::Time::now();

	msg.left_image.header.stamp = msg.header.stamp;
	msg.right_image.header.stamp = msg.header.stamp;

	msg.left_image.height = frame->height;
	msg.left_image.width = frame->width;
	msg.left_image.encoding = sensor_msgs::image_encodings::MONO8;
	msg.left_image.step = frame->width;

	msg.right_image.height = frame->height;
	msg.right_image.width = frame->width;
	msg.right_image.encoding = sensor_msgs::image_encodings::MONO8;
	msg.right_image.step = frame->width;


	msg.left_image2.height = frame->height;
	msg.left_image2.width = frame->width;
	msg.left_image2.encoding = sensor_msgs::image_encodings::MONO8;
	msg.left_image2.step = frame->width;

	msg.right_image2.height = frame->height;
	msg.right_image2.width = frame->width;
	msg.right_image2.encoding = sensor_msgs::image_encodings::MONO8;
	msg.right_image2.step = frame->width;

	msg.left_image3.height = frame->height;
	msg.left_image3.width = frame->width;
	msg.left_image3.encoding = sensor_msgs::image_encodings::MONO8;
	msg.left_image3.step = frame->width;

	msg.right_image3.height = frame->height;
	msg.right_image3.width = frame->width;
	msg.right_image3.encoding = sensor_msgs::image_encodings::MONO8;
	msg.right_image3.step = frame->width;

	msg.left_image4.height = frame->height;
	msg.left_image4.width = frame->width;
	msg.left_image4.encoding = sensor_msgs::image_encodings::MONO8;
	msg.left_image4.step = frame->width;

	msg.right_image4.height = frame->height;
	msg.right_image4.width = frame->width;
	msg.right_image4.encoding = sensor_msgs::image_encodings::MONO8;
	msg.right_image4.step = frame->width;

	unsigned frame_size = frame->height * frame->width * 2;

	// read the IMU data
	int16_t zero = 0;
	uint16_t cam_id = 0;


	for (unsigned i = 0; i < frame->height; i += 1) {

		uint16_t count = ShortSwap(static_cast<uint16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 0)]);

		//detect cam_id in first row
		if(i == 0){
			cam_id = count>>14;
		}

		//double temp  = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 1)]));

		double acc_x = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 2)])) /
			       (acc_scale_factor / 9.81);
		double acc_y = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 3)])) /
			       (acc_scale_factor / 9.81);
		double acc_z = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 4)])) /
			       (acc_scale_factor / 9.81);

		double gyr_x = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 5)]) /
				      (gyr_scale_factor / deg2rad));
		double gyr_y = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 6)]) /
				      (gyr_scale_factor / deg2rad));
		double gyr_z = double(ShortSwap(static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + 7)]) /
				      (gyr_scale_factor / deg2rad));

		//if (!(acc_x == acc_x_prev && acc_y == acc_y_prev && acc_z == acc_z_prev && gyr_x == gyr_x_prev && gyr_y == gyr_y_prev
		//      && gyr_z == gyr_z_prev)) {
		if(!(count == count_prev)){
//            printf("\nacc x %+02.4f ", acc_x);
//            printf("acc y %+04.4f ", acc_y);
//            printf("acc z %+04.4f ", acc_z);
//            printf("gyr x %+04.4f ", gyr_x);
//            printf("gyr y %+04.4f ", gyr_y);
//            printf("gyr z %+04.4f\n", gyr_z);
 //             printf("count %d \n", count);
 //             printf("temp %d \n", temp);
//printf("cam_id %d \n", cam_id);

			sensor_msgs::Imu imu_msg;
			imu_msg.linear_acceleration.x = -acc_y;
			imu_msg.linear_acceleration.y = -acc_x;
			imu_msg.linear_acceleration.z = -acc_z;

			imu_msg.angular_velocity.x = -gyr_y;
			imu_msg.angular_velocity.y = -gyr_x;
			imu_msg.angular_velocity.z = -gyr_z;

			msg.imu.push_back(imu_msg);

			count_prev = count;
			//acc_x_prev = acc_x;
			//acc_y_prev = acc_y;
			//acc_z_prev = acc_z;
			//gyr_x_prev = gyr_x;
			//gyr_y_prev = gyr_y;
			//gyr_z_prev = gyr_z;
		}

		for (unsigned j = 0; j < 8; j++) {
			static_cast<int16_t *>(frame->data)[int((i + 1) * frame->width - 8 + j)] = zero;
		}
	}

	// linearly interpolate the time stamps of the imu messages
	ros::Duration elapsed = msg.header.stamp - last_time;
	last_time = msg.header.stamp;

	ros::Time stamp_time = msg.header.stamp;

	printf("time elapsed: %f   ", elapsed.toSec());
	printf("framerate: %f   ", 1.0f/elapsed.toSec());
	printf("%lu imu messages\n", msg.imu.size());

	for (unsigned i = 0; i < msg.imu.size(); i++) {
		msg.imu[i].header.stamp = stamp_time - elapsed +ros::Duration(elapsed * (double(i) / msg.imu.size()));
	}

	// read the image data
	/*if (user_data->hflip) {
		for (unsigned i = frame_size; i > 0; i -= 2) {
			msg.left_image.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}

	} else {
		for (unsigned i = 0; i < frame_size; i += 2) {
			msg.left_image.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}
	}*/
	// read the image data
	if (cam_id==0) { //select_cam = 1 +2
		for (unsigned i = 0; i < frame_size; i += 2) {
			msg.left_image.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}
		msg.imu.clear();

	} 

	if (cam_id==1) { //select_cam = 3 +4
		for (unsigned i = 0; i < frame_size; i += 2) {
			msg.left_image2.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image2.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}
		msg.imu.clear();

	} 

	if (cam_id==2) { //select_cam = 4 +5
		for (unsigned i = 0; i < frame_size; i += 2) {
			msg.left_image3.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image3.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}
		msg.imu.clear();

	} 

	if (cam_id==3) { //select_cam = 6 +7
		for (unsigned i = 0; i < frame_size; i += 2) {
			msg.left_image4.data.push_back((static_cast<unsigned char *>(frame->data)[i])); // left image
			msg.right_image4.data.push_back((static_cast<unsigned char *>(frame->data)[i + 1])); // right image
		}
		user_data->pub.publish(msg);

		msg.left_image.data.clear();
		msg.right_image.data.clear();
		msg.left_image2.data.clear();
		msg.right_image2.data.clear();
		msg.left_image3.data.clear();
		msg.right_image3.data.clear();
		msg.left_image4.data.clear();
		msg.right_image4.data.clear();
		msg.imu.clear();
	}

	//user_data->pub.publish(msg);
}

int set_param(Serial_Port &sp, const char* name, float val) {
	mavlink_message_t msg;
	char name_buf[16] = {};

	uint8_t local_sys = 1;
	uint8_t local_comp = 1;

	uint8_t target_sys = 99;
	uint8_t target_comp = 55;

	// SETCALIB
	strncpy(name_buf, name, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
	mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys, target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
	int ret = sp.write_message(msg);
	if (ret <= 0) {
		printf("ret: %d\n", ret);
		return ret;
	}

	// another time, just so we maximize chances things actually go through
	strncpy(name_buf, name, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
	mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys, target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
	ret = sp.write_message(msg);
	if (ret <= 0) {
		printf("ret: %d\n", ret);
		return ret;
	}

	return 0;
}

int set_calibration() {

	Eigen::Matrix3d H0;
	Eigen::Matrix3d H1;
	double f_new;
	Eigen::Vector2d p0_new;
	Eigen::Vector2d p1_new;

	// CAMERA 0
	uvc_ros_driver::FPGACalibration cam0;
	cam0.projection_model_.type_ = cam0.projection_model_.PINHOLE;
	cam0.projection_model_.focal_length_u_ = 717.40f;
	cam0.projection_model_.focal_length_v_ = 717.49f;
	cam0.projection_model_.principal_point_u_ = 327.93f;
	cam0.projection_model_.principal_point_v_ = 270.17f;
	cam0.projection_model_.k1_ = 0.1783f;
	cam0.projection_model_.k2_ = -0.3191f;
	cam0.projection_model_.r1_ = 0.00f;
	cam0.projection_model_.r2_ = 0.00f;
	cam0.projection_model_.R_[0] = 1.0f;
	cam0.projection_model_.R_[1] = 0.0f;
	cam0.projection_model_.R_[2] = 0.0f;
	cam0.projection_model_.R_[3] = 0.0f;
	cam0.projection_model_.R_[4] = 1.0f;
	cam0.projection_model_.R_[5] = 0.0f;
	cam0.projection_model_.R_[6] = 0.0f;
	cam0.projection_model_.R_[7] = 0.0f;
	cam0.projection_model_.R_[8] = 1.0f;
	cam0.projection_model_.t_[0] = 0.0f;
	cam0.projection_model_.t_[1] = 0.0f;
	cam0.projection_model_.t_[2] = 0.0f;

	// CAMERA 1
	uvc_ros_driver::FPGACalibration cam1;
	cam1.projection_model_.type_ = cam1.projection_model_.PINHOLE;
	cam1.projection_model_.focal_length_u_ = 714.46f;
	cam1.projection_model_.focal_length_v_ = 714.34f;
	cam1.projection_model_.principal_point_u_ = 359.40f;
	cam1.projection_model_.principal_point_v_ = 242.25f;
	cam1.projection_model_.k1_ = 0.2031f;
	cam1.projection_model_.k2_ = -0.5438f;
	cam1.projection_model_.r1_ = 0.00f;
	cam1.projection_model_.r2_ = 0.00f;
	cam1.projection_model_.R_[0] = 0.9999f;
	cam1.projection_model_.R_[1] = -0.0122f;
	cam1.projection_model_.R_[2] = 0.0009f;
	cam1.projection_model_.R_[3] = 0.0122f;
	cam1.projection_model_.R_[4] = 0.9999f;
	cam1.projection_model_.R_[5] = -0.0008f;
	cam1.projection_model_.R_[6] = -0.0009f;
	cam1.projection_model_.R_[7] = 0.0008f;
	cam1.projection_model_.R_[8] = 0.9999f;
	cam1.projection_model_.t_[0] = -0.0685f;
	cam1.projection_model_.t_[1] = 0.0002f;
	cam1.projection_model_.t_[2] = 0.0004f;

	StereoHomography h(cam0, cam1);
	h.getHomography(H0, H1, f_new, p0_new, p1_new);

	Eigen::Matrix3d H2;
	Eigen::Matrix3d H3;
	double f_new2;
	Eigen::Vector2d p0_new2;
	Eigen::Vector2d p1_new2;

	// CAMERA 2
	uvc_ros_driver::FPGACalibration cam2;
	cam2.projection_model_.type_ = cam2.projection_model_.PINHOLE;
	cam2.projection_model_.focal_length_u_ = 721.76f;
	cam2.projection_model_.focal_length_v_ = 721.82f;
	cam2.projection_model_.principal_point_u_ = 364.32f;
	cam2.projection_model_.principal_point_v_ = 239.03f;
	cam2.projection_model_.k1_ = 0.1938f;
	cam2.projection_model_.k2_ = -0.5520f;
	cam2.projection_model_.r1_ = 0.00f;
	cam2.projection_model_.r2_ = 0.00f;
	cam2.projection_model_.R_[0] = 1.0f;
	cam2.projection_model_.R_[1] = 0.0f;
	cam2.projection_model_.R_[2] = 0.0f;
	cam2.projection_model_.R_[3] = 0.0f;
	cam2.projection_model_.R_[4] = 1.0f;
	cam2.projection_model_.R_[5] = 0.0f;
	cam2.projection_model_.R_[6] = 0.0f;
	cam2.projection_model_.R_[7] = 0.0f;
	cam2.projection_model_.R_[8] = 1.0f;
	cam2.projection_model_.t_[0] = 0.0f;
	cam2.projection_model_.t_[1] = 0.0f;
	cam2.projection_model_.t_[2] = 0.0f;

	// CAMERA 3
	uvc_ros_driver::FPGACalibration cam3;
	cam3.projection_model_.type_ = cam3.projection_model_.PINHOLE;
	cam3.projection_model_.focal_length_u_ = 716.53f;
	cam3.projection_model_.focal_length_v_ = 716.39f;
	cam3.projection_model_.principal_point_u_ = 341.14f;
	cam3.projection_model_.principal_point_v_ = 208.92f;
	cam3.projection_model_.k1_ = 0.1863f;
	cam3.projection_model_.k2_ = -0.4791f;
	cam3.projection_model_.r1_ = 0.00f;
	cam3.projection_model_.r2_ = 0.00f;
	cam3.projection_model_.R_[0] = 0.9999f;
	cam3.projection_model_.R_[1] = 0.0011f;
	cam3.projection_model_.R_[2] = -0.0041f;
	cam3.projection_model_.R_[3] = -0.0011f;
	cam3.projection_model_.R_[4] = 0.9999f;
	cam3.projection_model_.R_[5] = 0.0000f;
	cam3.projection_model_.R_[6] = 0.0041f;
	cam3.projection_model_.R_[7] = -0.0000f;
	cam3.projection_model_.R_[8] = 0.9999f;
	cam3.projection_model_.t_[0] = -0.0685f;
	cam3.projection_model_.t_[1] = -0.0001f;
	cam3.projection_model_.t_[2] = -0.0011f;

	StereoHomography h2(cam2, cam3);
	h2.getHomography(H2, H3, f_new2, p0_new2, p1_new2);

	Eigen::Matrix3d H4;
	Eigen::Matrix3d H5;
	double f_new3;
	Eigen::Vector2d p0_new3;
	Eigen::Vector2d p1_new3;


	// CAMERA 4
	uvc_ros_driver::FPGACalibration cam4;
	cam4.projection_model_.type_ = cam4.projection_model_.PINHOLE;
	cam4.projection_model_.focal_length_u_ = 713.35f;
	cam4.projection_model_.focal_length_v_ = 714.17f;
	cam4.projection_model_.principal_point_u_ = 306.68f;
	cam4.projection_model_.principal_point_v_ = 271.44f;
	cam4.projection_model_.k1_ = 0.2279f;
	cam4.projection_model_.k2_ = -0.5868f;
	cam4.projection_model_.r1_ = 0.00f;
	cam4.projection_model_.r2_ = 0.00f;
	cam4.projection_model_.R_[0] = 1.0f;
	cam4.projection_model_.R_[1] = 0.0f;
	cam4.projection_model_.R_[2] = 0.0f;
	cam4.projection_model_.R_[3] = 0.0f;
	cam4.projection_model_.R_[4] = 1.0f;
	cam4.projection_model_.R_[5] = 0.0f;
	cam4.projection_model_.R_[6] = 0.0f;
	cam4.projection_model_.R_[7] = 0.0f;
	cam4.projection_model_.R_[8] = 1.0f;
	cam4.projection_model_.t_[0] = 0.0f;
	cam4.projection_model_.t_[1] = 0.0f;
	cam4.projection_model_.t_[2] = 0.0f;

	// CAMERA 5
	uvc_ros_driver::FPGACalibration cam5;
	cam5.projection_model_.type_ = cam5.projection_model_.PINHOLE;
	cam5.projection_model_.focal_length_u_ = 718.52f;
	cam5.projection_model_.focal_length_v_ = 718.72f;
	cam5.projection_model_.principal_point_u_ = 355.84f;
	cam5.projection_model_.principal_point_v_ = 222.30f;
	cam5.projection_model_.k1_ = 0.2044f;
	cam5.projection_model_.k2_ = -0.5206f;
	cam5.projection_model_.r1_ = 0.00f;
	cam5.projection_model_.r2_ = 0.00f;
	cam5.projection_model_.R_[0] = 0.9995f;
	cam5.projection_model_.R_[1] = -0.0298f;
	cam5.projection_model_.R_[2] = -0.0094f;
	cam5.projection_model_.R_[3] = 0.0298f;
	cam5.projection_model_.R_[4] = 0.9995f;
	cam5.projection_model_.R_[5] = -0.0002f;
	cam5.projection_model_.R_[6] = 0.0094f;
	cam5.projection_model_.R_[7] = 0.0000f;
	cam5.projection_model_.R_[8] = 0.9999f;
	cam5.projection_model_.t_[0] = -0.0687f;
	cam5.projection_model_.t_[1] = -0.0025f;
	cam5.projection_model_.t_[2] = -0.0007f;

	StereoHomography h3(cam4, cam5);
	h3.getHomography(H4, H5, f_new3, p0_new3, p1_new3);

	Eigen::Matrix3d H6;
	Eigen::Matrix3d H7;
	double f_new4;
	Eigen::Vector2d p0_new4;
	Eigen::Vector2d p1_new4;


	// CAMERA 6
	uvc_ros_driver::FPGACalibration cam6;
	cam6.projection_model_.type_ = cam6.projection_model_.PINHOLE;
	cam6.projection_model_.focal_length_u_ = 713.87f;
	cam6.projection_model_.focal_length_v_ = 714.38f;
	cam6.projection_model_.principal_point_u_ = 285.76f;
	cam6.projection_model_.principal_point_v_ = 239.50f;
	cam6.projection_model_.k1_ = 0.2581f;
	cam6.projection_model_.k2_ = -0.8082f;
	cam6.projection_model_.r1_ = 0.00f;
	cam6.projection_model_.r2_ = 0.00f;
	cam6.projection_model_.R_[0] = 1.0f;
	cam6.projection_model_.R_[1] = 0.0f;
	cam6.projection_model_.R_[2] = 0.0f;
	cam6.projection_model_.R_[3] = 0.0f;
	cam6.projection_model_.R_[4] = 1.0f;
	cam6.projection_model_.R_[5] = 0.0f;
	cam6.projection_model_.R_[6] = 0.0f;
	cam6.projection_model_.R_[7] = 0.0f;
	cam6.projection_model_.R_[8] = 1.0f;
	cam6.projection_model_.t_[0] = 0.0f;
	cam6.projection_model_.t_[1] = 0.0f;
	cam6.projection_model_.t_[2] = 0.0f;

	// CAMERA 7
	uvc_ros_driver::FPGACalibration cam7;
	cam7.projection_model_.type_ = cam7.projection_model_.PINHOLE;
	cam7.projection_model_.focal_length_u_ = 709.88f;
	cam7.projection_model_.focal_length_v_ = 709.64f;
	cam7.projection_model_.principal_point_u_ = 357.24f;
	cam7.projection_model_.principal_point_v_ = 190.98f;
	cam7.projection_model_.k1_ = 0.1800f;
	cam7.projection_model_.k2_ = -0.3928f;
	cam7.projection_model_.r1_ = 0.00f;
	cam7.projection_model_.r2_ = 0.00f;
	cam7.projection_model_.R_[0] = 0.9996f;
	cam7.projection_model_.R_[1] = -0.0166f;
	cam7.projection_model_.R_[2] = -0.0224f;
	cam7.projection_model_.R_[3] = 0.0166f;
	cam7.projection_model_.R_[4] = 0.9998f;
	cam7.projection_model_.R_[5] = 0.0040f;
	cam7.projection_model_.R_[6] = 0.0223f;
	cam7.projection_model_.R_[7] = -0.0044f;
	cam7.projection_model_.R_[8] = 0.9997f;
	cam7.projection_model_.t_[0] = -0.0691f;
	cam7.projection_model_.t_[1] = -0.0003f;
	cam7.projection_model_.t_[2] = -0.0001f;

	StereoHomography h4(cam6, cam7);
	h4.getHomography(H6, H7, f_new4, p0_new4, p1_new4);

	Serial_Port sp = Serial_Port("/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DB00W5GV-if00-port0", 115200);

	sp.open_serial();

	// Set all parameters here
	set_param(sp, "PARAM_CCX_CAM1", p0_new[0]);
	set_param(sp, "PARAM_CCY_CAM1", p0_new[1]);
	set_param(sp, "PARAM_FCX_CAM1", f_new);
	set_param(sp, "PARAM_FCY_CAM1", f_new);
	set_param(sp, "PARAM_KC1_CAM1", cam0.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM1", cam0.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM1", H0(0, 0));
	set_param(sp, "PARAM_H12_CAM1", H0(0, 1));
	set_param(sp, "PARAM_H13_CAM1", H0(0, 2));
	set_param(sp, "PARAM_H21_CAM1", H0(1, 0));
	set_param(sp, "PARAM_H22_CAM1", H0(1, 1));
	set_param(sp, "PARAM_H23_CAM1", H0(1, 2));
	set_param(sp, "PARAM_H31_CAM1", H0(2, 0));
	set_param(sp, "PARAM_H32_CAM1", H0(2, 1));
	set_param(sp, "PARAM_H33_CAM1", H0(2, 2));

	set_param(sp, "PARAM_CCX_CAM2", p1_new[0]);
	set_param(sp, "PARAM_CCY_CAM2", p1_new[1]);
	set_param(sp, "PARAM_FCX_CAM2", f_new);
	set_param(sp, "PARAM_FCY_CAM2", f_new);
	set_param(sp, "PARAM_KC1_CAM2", cam1.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM2", cam1.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM2", H1(0, 0));
	set_param(sp, "PARAM_H12_CAM2", H1(0, 1));
	set_param(sp, "PARAM_H13_CAM2", H1(0, 2));
	set_param(sp, "PARAM_H21_CAM2", H1(1, 0));
	set_param(sp, "PARAM_H22_CAM2", H1(1, 1));
	set_param(sp, "PARAM_H23_CAM2", H1(1, 2));
	set_param(sp, "PARAM_H31_CAM2", H1(2, 0));
	set_param(sp, "PARAM_H32_CAM2", H1(2, 1));
	set_param(sp, "PARAM_H33_CAM2", H1(2, 2));

	set_param(sp, "PARAM_CCX_CAM3", p0_new2[0]);
	set_param(sp, "PARAM_CCY_CAM3", p0_new2[1]);
	set_param(sp, "PARAM_FCX_CAM3", f_new2);
	set_param(sp, "PARAM_FCY_CAM3", f_new2);
	set_param(sp, "PARAM_KC1_CAM3", cam2.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM3", cam2.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM3", H2(0, 0));
	set_param(sp, "PARAM_H12_CAM3", H2(0, 1));
	set_param(sp, "PARAM_H13_CAM3", H2(0, 2));
	set_param(sp, "PARAM_H21_CAM3", H2(1, 0));
	set_param(sp, "PARAM_H22_CAM3", H2(1, 1));
	set_param(sp, "PARAM_H23_CAM3", H2(1, 2));
	set_param(sp, "PARAM_H31_CAM3", H2(2, 0));
	set_param(sp, "PARAM_H32_CAM3", H2(2, 1));
	set_param(sp, "PARAM_H33_CAM3", H2(2, 2));

	set_param(sp, "PARAM_CCX_CAM4", p1_new2[0]);
	set_param(sp, "PARAM_CCY_CAM4", p1_new2[1]);
	set_param(sp, "PARAM_FCX_CAM4", f_new2);
	set_param(sp, "PARAM_FCY_CAM4", f_new2);
	set_param(sp, "PARAM_KC1_CAM4", cam3.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM4", cam3.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM4", H3(0, 0));
	set_param(sp, "PARAM_H12_CAM4", H3(0, 1));
	set_param(sp, "PARAM_H13_CAM4", H3(0, 2));
	set_param(sp, "PARAM_H21_CAM4", H3(1, 0));
	set_param(sp, "PARAM_H22_CAM4", H3(1, 1));
	set_param(sp, "PARAM_H23_CAM4", H3(1, 2));
	set_param(sp, "PARAM_H31_CAM4", H3(2, 0));
	set_param(sp, "PARAM_H32_CAM4", H3(2, 1));
	set_param(sp, "PARAM_H33_CAM4", H3(2, 2));

	set_param(sp, "PARAM_CCX_CAM5", p0_new3[0]);
	set_param(sp, "PARAM_CCY_CAM5", p0_new3[1]);
	set_param(sp, "PARAM_FCX_CAM5", f_new3);
	set_param(sp, "PARAM_FCY_CAM5", f_new3);
	set_param(sp, "PARAM_KC1_CAM5", cam4.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM5", cam4.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM5", H4(0, 0));
	set_param(sp, "PARAM_H12_CAM5", H4(0, 1));
	set_param(sp, "PARAM_H13_CAM5", H4(0, 2));
	set_param(sp, "PARAM_H21_CAM5", H4(1, 0));
	set_param(sp, "PARAM_H22_CAM5", H4(1, 1));
	set_param(sp, "PARAM_H23_CAM5", H4(1, 2));
	set_param(sp, "PARAM_H31_CAM5", H4(2, 0));
	set_param(sp, "PARAM_H32_CAM5", H4(2, 1));
	set_param(sp, "PARAM_H33_CAM5", H4(2, 2));

	set_param(sp, "PARAM_CCX_CAM6", p1_new3[0]);
	set_param(sp, "PARAM_CCY_CAM6", p1_new3[1]);
	set_param(sp, "PARAM_FCX_CAM6", f_new3);
	set_param(sp, "PARAM_FCY_CAM6", f_new3);
	set_param(sp, "PARAM_KC1_CAM6", cam5.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM6", cam5.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM6", H5(0, 0));
	set_param(sp, "PARAM_H12_CAM6", H5(0, 1));
	set_param(sp, "PARAM_H13_CAM6", H5(0, 2));
	set_param(sp, "PARAM_H21_CAM6", H5(1, 0));
	set_param(sp, "PARAM_H22_CAM6", H5(1, 1));
	set_param(sp, "PARAM_H23_CAM6", H5(1, 2));
	set_param(sp, "PARAM_H31_CAM6", H5(2, 0));
	set_param(sp, "PARAM_H32_CAM6", H5(2, 1));
	set_param(sp, "PARAM_H33_CAM6", H5(2, 2));

	set_param(sp, "PARAM_CCX_CAM7", p0_new4[0]);
	set_param(sp, "PARAM_CCY_CAM7", p0_new4[1]);
	set_param(sp, "PARAM_FCX_CAM7", f_new4);
	set_param(sp, "PARAM_FCY_CAM7", f_new4);
	set_param(sp, "PARAM_KC1_CAM7", cam6.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM7", cam6.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM7", H6(0, 0));
	set_param(sp, "PARAM_H12_CAM7", H6(0, 1));
	set_param(sp, "PARAM_H13_CAM7", H6(0, 2));
	set_param(sp, "PARAM_H21_CAM7", H6(1, 0));
	set_param(sp, "PARAM_H22_CAM7", H6(1, 1));
	set_param(sp, "PARAM_H23_CAM7", H6(1, 2));
	set_param(sp, "PARAM_H31_CAM7", H6(2, 0));
	set_param(sp, "PARAM_H32_CAM7", H6(2, 1));
	set_param(sp, "PARAM_H33_CAM7", H6(2, 2));

	set_param(sp, "PARAM_CCX_CAM8", p1_new4[0]);
	set_param(sp, "PARAM_CCY_CAM8", p1_new4[1]);
	set_param(sp, "PARAM_FCX_CAM8", f_new4);
	set_param(sp, "PARAM_FCY_CAM8", f_new4);
	set_param(sp, "PARAM_KC1_CAM8", cam7.projection_model_.k1_);
	set_param(sp, "PARAM_KC2_CAM8", cam7.projection_model_.k2_);
	set_param(sp, "PARAM_H11_CAM8", H7(0, 0));
	set_param(sp, "PARAM_H12_CAM8", H7(0, 1));
	set_param(sp, "PARAM_H13_CAM8", H7(0, 2));
	set_param(sp, "PARAM_H21_CAM8", H7(1, 0));
	set_param(sp, "PARAM_H22_CAM8", H7(1, 1));
	set_param(sp, "PARAM_H23_CAM8", H7(1, 2));
	set_param(sp, "PARAM_H31_CAM8", H7(2, 0));
	set_param(sp, "PARAM_H32_CAM8", H7(2, 1));
	set_param(sp, "PARAM_H33_CAM8", H7(2, 2));

	set_param(sp, "STEREO_P1", 16.0f);
	set_param(sp, "STEREO_P2", 250.0f);
	set_param(sp, "STEREO_LRCHK", 4.0f);
	set_param(sp, "STEREO_THOLD", 100.0f);
	set_param(sp, "STEREO_MASK", 0.0f);

	//set_param(sp, "RESETCALIB", 1.0f);
	set_param(sp, "SETCALIB", 1.0f);
	set_param(sp, "STEREO_ENABLE", 1.0f);

	set_param(sp, "RESETMT9V034", 1.0f);

	//std::cout<<"H0: "<<H0<<"\n";
	//std::cout<<"H1: "<<H1<<"\n";
	//ROS_INFO("fnew %f",f_new);

	sp.close_serial();

	return 0;
}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "uvc_ros_driver");
	ros::NodeHandle nh("~");  // private nodehandle

	last_time = ros::Time::now();

	UserData user_data;
	user_data.pub = nh.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor", 1);

	nh.param<bool>("hflip", user_data.hflip, false);
	nh.param<bool>("serialconfig", user_data.serialconfig, false);
	//nh.getParam("serialconfig",user_data.serialconfig);

	ros::Publisher serial_nr_pub = nh.advertise<std_msgs::String>("/vio_sensor/device_serial_nr", 1, true);

	//open serial port and write config to FPGA
	if (user_data.serialconfig) {
	  set_calibration();
	}

	uvc_context_t *ctx;
	uvc_device_t *dev;
	uvc_device_handle_t *devh;
	uvc_stream_ctrl_t ctrl;
	uvc_error_t res;

	/* Initialize a UVC service context. Libuvc will set up its own libusb
	 * context. Replace NULL with a libusb_context pointer to run libuvc
	 * from an existing libusb context. */
	res = uvc_init(&ctx, NULL);

	if (res < 0) {
		uvc_perror(res, "uvc_init");
		return res;
	}

	/* Locates the first attached UVC device, stores in dev */
	res = uvc_find_device(ctx, &dev, 0x04b4, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

	if (res < 0) {
		uvc_perror(res, "uvc_find_device"); /* no devices found */
		ROS_ERROR("No devices found");

	} else {
		ROS_INFO("Device found");

		/* Try to open the device: requires exclusive access */
		res = uvc_open(dev, &devh);

		if (res < 0) {
			uvc_perror(res, "uvc_open"); /* unable to open device */
			ROS_ERROR("Unable to open the device");

		} else {
			ROS_INFO("Device opened");

			uvc_device_descriptor_t *desc;
			uvc_get_device_descriptor(dev, &desc);
			std::stringstream serial_nr_ss;
			serial_nr_ss << desc->idVendor << "_" << desc->idProduct << "_TODO_SERIAL_NR";
			uvc_free_device_descriptor(desc);
			std_msgs::String str_msg;
			str_msg.data = serial_nr_ss.str();
			serial_nr_pub.publish(str_msg);

			/* Try to negotiate a 640x480 30 fps YUYV stream profile */
			res = uvc_get_stream_ctrl_format_size(devh, &ctrl, /* result stored in ctrl */
							      UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
							      640, 480, 30 /* width, height, fps */
							     );

			if (res < 0) {
				uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
				ROS_ERROR("Device doesn't provide a matching stream");

			} else {

				/*if (!repeatedStart(devh, ctrl)) {
					ROS_ERROR("Failed to get stream from the camera. Try powercycling the device");
					return -1;
				}*/

				/* Start the video stream. The library will call user function cb:
				 *   cb(frame, (void*) vio_sensor_pub)
				 */
				res = uvc_start_streaming(devh, &ctrl, uvc_cb, &user_data, 0);

				if (res < 0) {
					uvc_perror(res, "start_streaming"); /* unable to start stream */
					ROS_ERROR("Failed to start stream");

				} else {

					ros::spin();

					/* End the stream. Blocks until last callback is serviced */
					uvc_stop_streaming(devh);
					ROS_INFO("Done streaming.");
				}
			}

			/* Release our handle on the device */
			uvc_close(devh);
			ROS_INFO("Device closed");
		}

		/* Release the device descriptor */
		uvc_unref_device(dev);
	}

	/* Close the UVC context. This closes and cleans up any existing device handles,
	 * and it closes the libusb context if one was not provided. */
	uvc_exit(ctx);
	ROS_INFO("UVC exited");
	return 0;
}
