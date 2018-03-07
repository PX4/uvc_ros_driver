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

namespace uvc {

static void callback(uvc_frame *frame, void *arg) {
  uvcROSDriver *obj = (uvcROSDriver *)arg;
  obj->uvc_cb(frame);
}

static bool myPairMax(std::pair<int, int> p, std::pair<int, int> p1) {
  // gratest value is supposed be on the second index
  return p.second < p1.second;
}

uvcROSDriver::~uvcROSDriver() {
  if (serial_port_open_) {
    setParam("CAMERA_ENABLE", 0.0f);
  }
  mavlink_message_t message;
  mavlink_param_value_t param;
  int count = 0;
  bool wait = 1;
  std::string str = "CAMERA_ENABLE";

  if (serial_port_open_) {
    while (wait) {
      int res = sp_.read_message(message);
      if (res == -1) {
        serial_port_open_ = false;
        break;
      }
      if (res != 0) {
        if (message.msgid == 22) {
          mavlink_msg_param_value_decode(&message, &param);
          if (str.compare(param.param_id) == 0 && param.param_value == 0) {
            wait = 0;
          } else {
            setParam("CAMERA_ENABLE", 0.0f);
          }
          // std::cout << "received id " << param.param_id << " value " <<
          // param.param_value <<" iteration " 				<<
          // (float)
          // count
          // <<
          // std::endl;
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

void uvcROSDriver::initDevice() {
  // initialize serial port
  // sp_ = Serial_Port("/dev/ttyUSB0", 115200);
  sp_ = Serial_Port("/dev/serial/by-id/usb-Cypress_FX3-if02", 115200);

  bool first_fault = true;
  int open = 0;
  while (true) {
    ros::spinOnce();
    if (!nh_.ok()) return;
    open = sp_.open_serial();

    if (open != -1) {
      serial_port_open_ = true;
      break;
    }
    if (first_fault) {
      ROS_ERROR(
          "Couldn't open serialport /dev/serial/by-id/usb-Cypress_FX3-if02. "
          "Will retry every second.");
      first_fault = false;
    }
    sleep(1.0);
  }

  // initialize camera image publisher
  constexpr int kCamQueueSize = 5;
  constexpr int kIMUQueueSize = 20;
  std::string prev_topic;
  for (size_t i = 0; i < n_cameras_; ++i) {
    const std::string topic = "cam_" + std::to_string(i) + "/";
    cam_raw_pubs_.emplace_back(
        it_.advertise(topic + "image_raw", kCamQueueSize));
    cam_info_pubs_.emplace_back(
        nh_.advertise<sensor_msgs::CameraInfo>(topic + "camera_info", 1000));

    if (i % 2) {
      cam_rect_pubs_.emplace_back(
          it_.advertise(prev_topic + "image_rect", kCamQueueSize));
      cam_disp_pubs_.emplace_back(
          it_.advertise(prev_topic + "image_depth", kCamQueueSize));

      imu_pubs_.emplace_back(
          nh_.advertise<sensor_msgs::Imu>(prev_topic + "imu", kIMUQueueSize));
      imu_pubs_.emplace_back(
          nh_.advertise<sensor_msgs::Imu>(topic + "imu", kIMUQueueSize));
    }
    prev_topic = topic;
  }

  info_cams_.resize(n_cameras_);

  // time translator
  constexpr int kSecondsToNanoSeconds = 1e9;
  device_time_translator_.reset(
      new cuckoo_time_translator::UnwrappedDeviceTimeTranslator(
          cuckoo_time_translator::ClockParameters(kSecondsToNanoSeconds),
          nh_.getNamespace(),
          cuckoo_time_translator::Defaults().setFilterAlgorithm(
              cuckoo_time_translator::FilterAlgorithm::ConvexHull)));

  // wait on heart beat
  std::cout << "Waiting on device.";
  fflush(stdout);
  mavlink_message_t message;
  if (serial_port_open_ && nh_.ok()) {
    int res = sp_.read_message(message);
    if (res == -1) {
      serial_port_open_ = false;
      return;
    }

    mavlink_heartbeat_t heartbeat;

    while (heartbeat.type != 9) {  // check for system type 9 heartbeat

      if (heartbeat.type != 9) {
        printf(".");
        fflush(stdout);
        usleep(50000);
      }
      if (message.msgid == 0) {
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.type == 9) {
          std::cout << std::endl;
          ROS_INFO("Got heartbeat from camera");
        }
      }
      if (serial_port_open_ && nh_.ok()) {
        int res = sp_.read_message(message);
        if (res == -1) {
          serial_port_open_ = false;
          return;
        }
      }
    }

    // set flag for completed initializiation
    device_initialized_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::startDevice() {
  if (device_initialized_) {
    setCalibration(camera_params_);

    // open uvc stream
    uvc_error_t res = initAndOpenUvc();
    // start stream
    past_ = ros::Time::now();
    res = uvc_start_streaming(devh_, &ctrl_, &callback, this, 0);

    setParam("CAMERA_ENABLE", float(camera_config_));

    printf("Waiting on stream");
    while (!uvc_cb_flag_ && ros::ok()) {
      printf(".");
      fflush(stdout);

      if (setParam("CAMERA_ENABLE", float(camera_config_)) == -1) {
        ROS_ERROR("Device not initialized!");
        return;
      }
      usleep(200000);
    }

  } else {
    ROS_ERROR("Device not initialized!");
  }
}

////////////////////////////////////////////////////////////////////////////////

int uvcROSDriver::setParam(const std::string &name, float val) {
  mavlink_message_t msg;
  char name_buf[16] = {};

  uint8_t local_sys = 1;
  uint8_t local_comp = 1;

  uint8_t target_sys = 99;
  uint8_t target_comp = 55;

  constexpr size_t kAttempts = 2;

  // SETCALIB
  // multiple attempts, just so we maximize chances things actually go through
  for (size_t i = 0; i < kAttempts; ++i) {
    strncpy(name_buf, name.c_str(), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys,
                               target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
    int ret = sp_.write_message(msg);

    if (ret <= 0) {
      printf("ret: %d\n", ret);
      return ret;
    }
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::sendCameraParam(const int camera_number, const double fx,
                                   const double fy, const Eigen::Vector2d &p0,
                                   const float k1, const float k2,
                                   const float r1, const float r2,
                                   const Eigen::Matrix3d &H) {
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

void uvcROSDriver::setCalibration(CameraParameters camParams) {
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
          std::max_element(homography_mapping_.begin(),
                           homography_mapping_.end(), myPairMax);

      // set homography number to number of camera pairs for now
      homography_size = n_cameras_ / 2;

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
    setParam("STEREO_FP_CAM1", 0.0f);
    setParam("STEREO_CE_CAM1", 0.0f);
    setParam("STEREO_RE_CAM1", 0.0f);
    setParam("STEREO_OF_CAM1", 0.0f);

    // setParam("COST_SHIFT", 2.0f);

    // setParam("CAMERA_AUTOEXP",0.0f);
    // setParam("CAMERA_EXP",480.0f);
    // setParam("CAMERA_AUTOG",0.0f);
    // setParam("CAMERA_GAIN",63.0f);

    setParam("STEREO_MP_01", 0.0f);
    setParam("STEREO_BAYER_D", 0.0f);
    setParam("IMU_ENABLE", (float)n_cameras_);
    setParam("ADIS_IMU", 0.0f);

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

    if (flip_) {
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
    } else {
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

  setParam("SETCALIB", float(set_calibration_));

  // std::cout << "Configuring cameras..." << std::endl;
  setParam("RESETMT9V034", 1.0f);
  setParam("RESETICM20608", 1.0f);
  // sleep(5);  // needed, fpga reconfigure cameras and restart time
  // std::cout << "Configuration completed." << std::endl;
  // last 4 bits activate the 4 camera pairs 0x01 = pair 1 only, 0x0F all 4
  // pairs
  // setParam("CAMERA_ENABLE", float(camera_config_));
}

////////////////////////////////////////////////////////////////////////////////
// NOTE: return error really necessary?
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

inline void uvcROSDriver::selectCameraInfo(int camera,
                                           sensor_msgs::CameraInfo **ci) {
  *ci = &info_cams_[camera];
}

bool uvcROSDriver::extractAndTranslateTimestamp(size_t offset,
                                                uvc_frame_t *frame,
                                                ros::Time *stamp) {
  // read out micro second timestamp
  constexpr size_t kTimestampOffset = 10;
  constexpr size_t kImagesPerFrame = 2;
  const uint8_t *raw_timestamp = &static_cast<uint8_t *>(
      frame
          ->data)[kImagesPerFrame * (offset + frame->width - kTimestampOffset)];
  uint64_t fpga_timestamp = (raw_timestamp[3] << 0) | (raw_timestamp[2] << 8) |
                            (raw_timestamp[1] << 16) | (raw_timestamp[0] << 24);

  if (fpga_timestamp == 0) {
    return false;
  }

  constexpr uint64_t kMicroSecondsToNanoSeconds = 1e3;
  fpga_timestamp *= kMicroSecondsToNanoSeconds;
  // only update on image timestamps
  if (offset == 0) {
    device_time_translator_->update(fpga_timestamp, ros::Time::now());
  }

  ROS_ERROR_STREAM("Time: " << fpga_timestamp);

  if (device_time_translator_->isReadyToTranslate()) {
    *stamp = device_time_translator_->translate(fpga_timestamp);
  } else {
    return false;
  }

  return true;
}

CamID uvcROSDriver::extractCamId(uvc_frame_t *frame) {
  constexpr uint8_t kMaxCams = 10;
  constexpr size_t kCamIdOffset = 8;
  constexpr size_t kCamIdShift = 4;
  constexpr size_t kImagesPerFrame = 2;
  constexpr size_t kRectOffset = 8;
  uint8_t raw_id =
      static_cast<int8_t *>(
          frame->data)[kImagesPerFrame * (frame->width - kCamIdOffset)] >>
      kCamIdShift;

  CamID cam_id;
  if (raw_id < kMaxCams) {
    cam_id.left_cam_num = kImagesPerFrame * raw_id;
    cam_id.right_cam_num = cam_id.left_cam_num + 1;
    cam_id.is_raw_images = true;
  } else {
    cam_id.left_cam_num = kImagesPerFrame * (raw_id - kRectOffset);
    cam_id.right_cam_num = cam_id.left_cam_num;
    cam_id.is_raw_images = false;
  }

  return cam_id;
}

uint8_t uvcROSDriver::extractImuId(uvc_frame_t *frame) {
  constexpr size_t kImuIdOffset = 8;
  constexpr uint8_t kImuIdMask = 0x0F;
  constexpr size_t kImagesPerFrame = 2;
  return static_cast<int8_t *>(
             frame->data)[kImagesPerFrame * (frame->width - kImuIdOffset)] &
         kImuIdMask;
}

uint8_t uvcROSDriver::extractImuCount(size_t offset, uvc_frame_t *frame) {
  constexpr size_t kImuCountOffset = 9;
  constexpr size_t kImagesPerFrame = 2;

  return static_cast<int8_t *>(
      frame->data)[kImagesPerFrame * (offset * frame->width - kImuCountOffset)];
}

bool uvcROSDriver::extractImuData(size_t offset, uvc_frame_t *frame,
                                  sensor_msgs::Imu *msg) {
  if (!extractAndTranslateTimestamp(offset, frame, &msg->header.stamp)) {
    return false;
  }

  msg->linear_acceleration.x =
      extractImuElementData(offset, ImuElement::AX, frame);
  msg->linear_acceleration.y =
      extractImuElementData(offset, ImuElement::AY, frame);
  msg->linear_acceleration.z =
      extractImuElementData(offset, ImuElement::AZ, frame);

  msg->angular_velocity.x =
      extractImuElementData(offset, ImuElement::RX, frame);
  msg->angular_velocity.y =
      extractImuElementData(offset, ImuElement::RY, frame);
  msg->angular_velocity.z =
      extractImuElementData(offset, ImuElement::RZ, frame);

  return true;
}

double uvcROSDriver::extractImuElementData(size_t offset, ImuElement element,
                                           uvc_frame_t *frame) {
  const int8_t *raw_data = &static_cast<int8_t *>(
      frame->data)[2 * (offset * frame->width - 8 + element)];
  double data = static_cast<double>((raw_data[1] << 0) | (raw_data[0] << 8));
  if (element == ImuElement::AX || element == ImuElement::AY ||
      element == ImuElement::AZ) {
    data /= (acc_scale_factor / 9.81);
  } else if (element == ImuElement::RX || element == ImuElement::RY ||
             element == ImuElement::RZ) {
    data /= (gyr_scale_factor / deg2rad);
  }
  return data;
}

void uvcROSDriver::extractImages(uvc_frame_t *frame,
                                 ait_ros_messages::VioSensorMsg *msg_vio) {
  // read the image data and separate the 2 images
  const size_t mixed_size = 2 * (frame->height * frame->width);
  const std::valarray<uint8_t> data(static_cast<uint8_t *>(frame->data),
                                    mixed_size);
  const std::valarray<uint8_t> left_val =
      data[std::slice(0, mixed_size / 2, 2)];
  const std::valarray<uint8_t> right_val =
      data[std::slice(1, mixed_size / 2, 2)];
  std::vector<uint8_t> left(std::begin(left_val), std::end(left_val));
  std::vector<uint8_t> right(std::begin(right_val), std::end(right_val));

  sensor_msgs::fillImage(msg_vio->left_image,
                         sensor_msgs::image_encodings::MONO8,  //
                         frame->height,                        // height
                         frame->width - 16,                    // width
                         frame->width,                         // stepSize
                         left.data());

  sensor_msgs::fillImage(msg_vio->right_image,
                         sensor_msgs::image_encodings::MONO8,  // BAYER_RGGB8,//
                         frame->height,                        // height
                         frame->width - 16,                    // width
                         frame->width,                         // stepSize
                         right.data());
}

////////////////////////////////////////////////////////////////////////////////

/* This callback function runs once per frame. Use it to perform any
  quick processing you need, or have it put the frame into your application's
  input queue.If this function takes too long, you'll start losing frames. */
void uvcROSDriver::uvc_cb(uvc_frame_t *frame) {
  // check if evrytstartedhing ok
  if (!ros::ok()) {
    return;
  }

  if (!uvc_cb_flag_) {
    std::cout << std::endl;
    ROS_INFO("Stream started");
  }
  // flag
  uvc_cb_flag_ = true;

  ait_ros_messages::VioSensorMsg msg_vio;

  unsigned frame_size = frame->height * frame->width * 2;

  const uint8_t imu_id = extractImuId(frame);

  static uint8_t prev_count = 0;

  ROS_WARN("\n NEW FRAME \n");

  // process the IMU data
  for (size_t i = 1; i < frame->height; ++i) {
    sensor_msgs::Imu msg_imu;

    const uint8_t count = extractImuCount(i, frame);
    ROS_ERROR_STREAM("Count: " << (size_t)count);
    if ((count != prev_count) && extractImuData(i, frame, &msg_imu)) {
      const sensor_msgs::Imu base_msg_imu = msg_imu;
      if (flip_) {
        msg_imu.linear_acceleration.x = base_msg_imu.linear_acceleration.y;
        msg_imu.linear_acceleration.y = base_msg_imu.linear_acceleration.x;
        msg_imu.linear_acceleration.z = -base_msg_imu.linear_acceleration.z;

        msg_imu.angular_velocity.x = base_msg_imu.angular_velocity.y;
        msg_imu.angular_velocity.y = base_msg_imu.angular_velocity.x;
        msg_imu.angular_velocity.z = -base_msg_imu.angular_velocity.z;

      } else {
        // TODO: check if correct
        msg_imu.linear_acceleration.x = -base_msg_imu.linear_acceleration.y;
        msg_imu.linear_acceleration.y = -base_msg_imu.linear_acceleration.x;
        msg_imu.linear_acceleration.z = -base_msg_imu.linear_acceleration.z;

        msg_imu.angular_velocity.x = -base_msg_imu.angular_velocity.y;
        msg_imu.angular_velocity.y = -base_msg_imu.angular_velocity.x;
        msg_imu.angular_velocity.z = -base_msg_imu.angular_velocity.z;
      }

      msg_vio.imu.push_back(msg_imu);

      imu_pubs_[imu_id].publish(msg_imu);

      prev_count = count;
    }
  }

  ROS_DEBUG("%lu imu messages", msg_vio.imu.size());
  ROS_DEBUG("imu id: %d ", imu_id);

  extractImages(frame, &msg_vio);

  const CamID cam_id = extractCamId(frame);

  if (cam_id.right_cam_num > n_cameras_) {
    return;
  }

  const bool raw_enabled = (camera_config_ & 0x001) != 0;
  const uint16_t frame_counter_cam = n_cameras_ < 9 ? 0 : 8;

  if ((cam_id.left_cam_num == frame_counter_cam) &&
      (cam_id.is_raw_images == raw_enabled)) {
    if (!extractAndTranslateTimestamp(0, frame, &frame_time_)) {
      ROS_ERROR("Invalid timestamp, dropping frame");
    }
    frameCounter_++;
  }

  msg_vio.header.stamp = frame_time_;
  msg_vio.left_image.header.stamp = frame_time_;
  msg_vio.right_image.header.stamp = frame_time_;

  if (cam_id.is_raw_images) {
    msg_vio.left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_optical_frame";
    msg_vio.right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_optical_frame";

    if ((cam_id.left_cam_num != 0) && (frameCounter_ % modulo_ != 0)) {
      return;
    }

    cam_raw_pubs_[cam_id.left_cam_num].publish(msg_vio.left_image);
    cam_raw_pubs_[cam_id.right_cam_num].publish(msg_vio.right_image);
  } else {
    msg_vio.left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_corrected_frame";
    msg_vio.right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_disparity_frame";

    // publish images
    cam_rect_pubs_[cam_id.left_cam_num].publish(msg_vio.left_image);
    cam_disp_pubs_[cam_id.right_cam_num].publish(msg_vio.right_image);
  }
}

} /* uvc */
