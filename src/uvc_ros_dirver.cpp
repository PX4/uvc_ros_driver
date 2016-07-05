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
 *      Author: nicolas, christoph, simone
 *
 *  The code below is based on the example provided at
 *  https://int80k.com/libuvc/doc/
 */

#include "uvc_ros_driver.h"

namespace uvc {

void uvcROSDriver::initDevice() {
  // initialize serial port
  sp_ = Serial_Port("/dev/ttyUSB0", 115200);

  if (enable_ait_vio_msg_) {
    // initialize vio msgs publishers
    switch (n_cameras_) {
      case 10:
        stereo_vio_5_pub_ =
            nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor_4", 5);
      case 8:
        stereo_vio_4_pub_ =
            nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor_3", 5);
      case 6:
        stereo_vio_3_pub_ =
            nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor_2", 5);
      case 4:
        stereo_vio_2_pub_ =
            nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor_1", 5);
      default:
        stereo_vio_1_pub_ =
            nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor_0", 5);
    }
  } else {
    // initialize camera image publisher
    switch (n_cameras_) {
      case 10:
        cam_9_pub_ = it_.advertise("cam_9", 5);
      case 9:
        cam_8_pub_ = it_.advertise("cam_8", 5);
      case 8:
        cam_7_pub_ = it_.advertise("cam_7", 5);
      case 7:
        cam_6_pub_ = it_.advertise("cam_6", 5);
      case 6:
        cam_5_pub_ = it_.advertise("cam_5", 5);
      case 5:
        cam_4_pub_ = it_.advertise("cam_4", 5);
      case 4:
        cam_3_pub_ = it_.advertise("cam_3", 5);
      case 3:
        cam_2_pub_ = it_.advertise("cam_2", 5);
      case 2:
        cam_1_pub_ = it_.advertise("cam_1", 5);
      default:
        cam_0_pub_ = it_.advertise("cam_0", 5);
    }
  }
  // initialize imu msg publisher
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("/vio_imu", 1);

  CameraParameters camParams = loadCustomCameraCalibration(path_to_config_);

  // set flag for completed initializiation
  device_initialized_ = true;
}

void uvcROSDriver::startDevice() {
  if (device_initialized_) {
    // open uvc stream
  } else {
    ROS_ERROR("Device not initialized!");
  }
}

////////////////////////////////////////////////////////////////////////////////

int uvcROSDriver::set_param(const char* name, float val) {
  mavlink_message_t msg;
  char name_buf[16] = {};

  uint8_t local_sys = 1;
  uint8_t local_comp = 1;

  uint8_t target_sys = 99;
  uint8_t target_comp = 55;

  // SETCALIB
  strncpy(name_buf, name, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys,
                             target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
  int ret = sp_.write_message(msg);
  if (ret <= 0) {
    printf("ret: %d\n", ret);
    return ret;
  }

  // another time, just so we maximize chances things actually go through
  strncpy(name_buf, name, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
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

void setCameraParam(const int camera_number, const double f,
                    const Eigen::Vector2d& p0, const float k1, const float k2,
                    const float r1, const float r2 const Eigen::Matrix3d& H) {
  std::string camera_name = "CAM" + std::to_string(camera_number);
  set_param("PARAM_CCX_" + camera_name, p0[0]);
  set_param("PARAM_CCY_" + camera_name, p0[1]);
  set_param("PARAM_FCX_" + camera_name, f);
  set_param("PARAM_FCY_" + camera_name, f);
  set_param("PARAM_KC1_" + camera_name, k1);
  set_param("PARAM_KC2_" + camera_name, k2);
  set_param("PARAM_P1_" + camera_name, r1);
  set_param("PARAM_P2_" + camera_name, r2);
  set_param("PARAM_H11_" + camera_name, H(0, 0));
  set_param("PARAM_H12_" + camera_name, H(0, 1));
  set_param("PARAM_H13_" + camera_name, H(0, 2));
  set_param("PARAM_H21_" + camera_name, H(1, 0));
  set_param("PARAM_H22_" + camera_name, H(1, 1));
  set_param("PARAM_H23_" + camera_name, H(1, 2));
  set_param("PARAM_H31_" + camera_name, H(2, 0));
  set_param("PARAM_H32_" + camera_name, H(2, 1));
  set_param("PARAM_H33_" + camera_name, H(2, 2));
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::set_calibration(CameraParameters camParams) {
  // uvc_ros_driver::FPGACalibration cams[n_cameras_];
  std::vector<uvc_ros_driver::FPGACalibration> cams;
  int stereo_number = 0;
  // TODO: find better way for this, general case not only stereo between
  // cam0->cam1, cam2->cam3
  for (int cam = 0; cam < n_cameras_; cam++) {
    uvc_ros_driver::FPGACalibration camera;
    camera.projection_model_.type_ =
        uvc_ros_driver::ProjectionModelTypes.PINHOLE;
    camera.projection_model_.focal_length_u_ = camParams.FocalLength[cam][0];
    camera.projection_model_.focal_length_v_ = camParams.FocalLength[cam][1];
    camera.projection_model_.principal_point_u_ =
        camParams.PrincipalPoint[cam][0];
    camera.projection_model_.principal_point_v_ =
        camParams.PrincipalPoint[cam][1];
    camera.projection_model_.k1_ = camParams.DistortionCoeffs[cam][0];
    camera.projection_model_.k2_ = camParams.DistortionCoeffs[cam][1];
    camera.projection_model_.r1_ = camParams.DistortionCoeffs[cam][2];
    cams[cam].projection_model_.r2_ = camParams.DistortionCoeffs[cam][3];
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
      cams[cam].projection_model_.R_[5] = 0.0f;
      cams[cam].projection_model_.R_[6] = 0.0f;
      cams[cam].projection_model_.R_[7] = 0.0f;
      cams[cam].projection_model_.R_[8] = 1.0f;
      cams[cam].projection_model_.t_[0] = 0.0f;
      cams[cam].projection_model_.t_[1] = 0.0f;
      cams[cam].projection_model_.t_[2] = 0.0f;
    }
    cams.push_back(camera);
  }

  // initialize vectors
  f_.resize(n_cameras_);
  p_.resize(n_cameras_);
  H_.resize(n_cameras_);

  // temp structures
  Eigen::Matrix3d H0;
  Eigen::Matrix3d H1;
  double f_new;
  Eigen::Vector2d p0_new, p1_new;
  std::pair indx;
  // TODO: reimplment this part for multiple stereo base line based on for ex.
  // camera 0
  for (size_t i = 0; i < homography_mapping.size(); i++) {
    indx = homography_mapping[i];
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

  sp_.open_serial();

  // Set all parameters here
  for (size_t i = 0; i < n_cameras_; i++) {
    setCameraParam(i, f_[i]], p[i], cams[i].projection_model_.k1_,
                   cams[i].projection_model_.k2_, cams[i].projection_model_.r1_,
                   cams[i].projection_model_.r2_, H_[i]);
  }

  set_param("STEREO_P1_CAM1", 16.0f);
  set_param("STEREO_P2_CAM1", 250.0f);
  set_param("STEREO_LR_CAM1", 4.0f);
  set_param("STEREO_TH_CAM1", 100.0f);

  set_param("STEREO_P1_CAM3", 8.0f);
  set_param("STEREO_P2_CAM3", 240.0f);
  set_param("STEREO_LR_CAM3", 4.0f);
  set_param("STEREO_TH_CAM3", 140.0f);

  set_param("STEREO_P1_CAM5", 8.0f);
  set_param("STEREO_P2_CAM5", 240.0f);
  set_param("STEREO_LR_CAM5", 4.0f);
  set_param("STEREO_TH_CAM5", 140.0f);

  set_param("STEREO_P1_CAM7", 16.0f);
  set_param("STEREO_P2_CAM7", 250.0f);
  set_param("STEREO_LR_CAM7", 4.0f);
  set_param("STEREO_TH_CAM7", 100.0f);

  set_param("CALIB_GAIN", 4300.0f);

  // TODO change userData with internal variables

  set_param("CAMERA_H_FLIP", float(flip_));
  // last 4 bits activate the 4 camera pairs 0x01 = pair 1 only, 0x0F all 4
  // pairs
  set_param("CAMERA_ENABLE", float(userData->cameraConfig));

  if (userData->setCalibration)
    set_param("RESETCALIB", 0.0f);
  else
    set_param("RESETCALIB", 1.0f);

  set_param("SETCALIB", float(userData->setCalibration));

  set_param("STEREO_ENABLE", float(userData->depthMap));

  set_param("RESETMT9V034", 1.0f);
}

////////////////////////////////////////////////////////////////////////////////

uvc_error_t uvcROSDriver::initAndOpenUvc() {
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

  } else {
    ROS_INFO("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev_, &devh_);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
      ROS_ERROR("Unable to open the device");
    }
    return res;
  }
}

////////////////////////////////////////////////////////////////////////////////

int16_t uvcROSDriver::ShortSwap(int16_t s) {
  unsigned char b1, b2;
  b1 = s & 255;
  b2 = (s >> 8) & 255;
  return (b1 << 8) + b2;
}

} /* uvc */
