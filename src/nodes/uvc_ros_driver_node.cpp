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
 *      Author: nicolas, christoph
 *
 *  The code below is based on the example provided at
 *https://int80k.com/libuvc/doc/
 */

#include "uvc_ros_driver.h"

static const double acc_scale_factor = 16384.0;
static const double gyr_scale_factor = 131.0;
static const double deg2rad = 2 * M_PI / 360.0;

// declare helper function
CameraParameters loadCustomCameraCalibration(const std::string calib_path);

// static double acc_x_prev, acc_y_prev, acc_z_prev, gyr_x_prev, gyr_y_prev,
// gyr_z_prev;
static uint16_t count_prev;
static ros::Time last_time;

bool request_shutdown_flag = false;
bool uvc_cb_flag = false;
int frameCounter = 0;
int calibrationMode = 0;
ros::Time frame_time;

// struct holding all data needed in the callback
struct UserData {
  ros::Publisher image_publisher_1;
  ros::Publisher image_publisher_2;
  ros::Publisher image_publisher_3;
  ros::Publisher image_publisher_4;
  ros::Publisher imu_publisher;
  bool flip;
  bool serialconfig;
  bool setCalibration;
  bool depthMap;
  int cameraConfig;
};

inline void deinterleave(const uint8_t *mixed, uint8_t *array1, uint8_t *array2,
                         size_t mixedLength, size_t imageWidth,
                         size_t imageHeight) {
// TODO: modify ARM NEON instruction to support imageWidth
#if defined __ARM_NEON__
  size_t vectors = mixedLength / 32;
  mixedLength %= 32;

  while (vectors-- > 0) {
    const uint8x16_t src0 = vld1q_u8(mixed);
    const uint8x16_t src1 = vld1q_u8(mixed + 16);
    const uint8x16x2_t dst = vuzpq_u8(src0, src1);
    vst1q_u8(array1, dst.val[0]);
    vst1q_u8(array2, dst.val[1]);
    mixed += 32;
    array1 += 16;
    array2 += 16;
  }

#endif
  int i = 0;
  int c = 0;

  while (c < imageWidth * imageHeight) {
    array1[c] = mixed[2 * i];
    array2[c] = mixed[2 * i + 1];
    i++;
    c++;

    if (c % (imageWidth) == 0) {
      i += 16;
    }
  }
}

// void writeFocalLengthToYaml() {
//   std::string package_path = ros::package::getPath("uvc_ros_driver");
//   std::string calibrationFile0_Path =
//       package_path + "/calib/stereoPair1_Parameters.yaml";
//
//   YAML::Node node =
//       YAML::LoadFile(calibrationFile0_Path);  // gets the root node
//   node["NewFocalLengths"]["1"] = "test";      // edit one of the nodes
//   std::ofstream fout(calibrationFile0_Path);
//   YAML::Emitter out;
//   out << YAML::Block << node;
//   fout << out.c_str();  // dump it back into the file
// }

CameraParameters loadCustomCameraCalibration(const std::string calib_path) {
  // load a camera calibration defined in the launch script
  try {
    YAML::Node YamlNode = YAML::LoadFile(calib_path);

    if (YamlNode.IsNull()) {
      printf("Failed to open camera calibration %s\n", calib_path.c_str());
      exit(-1);
    }

    return parseYaml(YamlNode);
  }
  catch (YAML::BadFile &e) {
    printf("Failed to open camera calibration %s\nException: %s\n",
           calib_path.c_str(), e.what());
    exit(-1);
  }
}

void mySigintHandler(int sig) {
  request_shutdown_flag = true;
  uvc_cb_flag = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "uvc_ros_driver", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");  // private nodehandle

  // signal(SIGINT, mySigintHandler);
  uvc::uvcROSDriver uvc_ros_driver(nh);

  last_time = ros::Time::now();

  // get params from launch file
  bool flip, set_calibration, depth_map;
  int camera_config;
  std::string calibrationFileName;
  // TODO: check if parameter exist
  nh.getParam("flip", flip);
  nh.getParam("setCalibration", set_calibration);
  nh.getParam("depthMap", depth_map);
  nh.getParam("cameraConfig", camera_config);
  nh.getParam("cameraConfigFile", calibrationFileName);
  nh.getParam("calibrationMode", calibrationMode);

  // read yaml calibration file from launch file parameter, file muss be located
  // in the calib folder
  std::string package_path = ros::package::getPath("uvc_ros_driver");
  std::string calibrationFile_Path =
      package_path + "/calib/" + calibrationFileName;
  CameraParameters camParams =
      loadCustomCameraCalibration(calibrationFile_Path);

  std::vector<std::pair<int, int>> homography_mapping;
  homography_mapping.push_back(std::make_pair(0, 1));

  uvc_ros_driver.setNumberOfCameras(2);
  uvc_ros_driver.setFlip(flip);
  uvc_ros_driver.setCalibrationParam(set_calibration);
  uvc_ros_driver.setUseOfDepthMap(depth_map);
  uvc_ros_driver.setCameraConfig(camera_config);
  uvc_ros_driver.setCameraParams(camParams);
  uvc_ros_driver.setHomographyMapping(homography_mapping);
  uvc_ros_driver.initDevice();

  // // open serial port and write config to FPGA
  // if (user_data.serialconfig) {
  //   set_calibration(&user_data, camParams);
  // }
  //
  // uvc_context_t *ctx;
  // uvc_device_t *dev;
  // uvc_device_handle_t *devh;
  // uvc_stream_ctrl_t ctrl;
  // uvc_error_t res;
  //
  // /* Initialize a UVC service context. Libuvc will set up its own libusb
  //  * context. Replace NULL with a libusb_context pointer to run libuvc
  //  * from an existing libusb context. */
  // res = uvc_init(&ctx, NULL);
  //
  // if (res < 0) {
  //   uvc_perror(res, "uvc_init");
  //   return res;
  // }
  //
  // /* Locates the first attached UVC device, stores in dev */
  // res = uvc_find_device(ctx, &dev, 0x04b4, 0, NULL); /* filter devices:
  //                                                       vendor_id,
  // product_id,
  //                                                       "serial_num" */
  //
  // if (res < 0) {
  //   uvc_perror(res, "uvc_find_device"); /* no devices found */
  //   ROS_ERROR("No devices found");
  //
  // } else {
  //   ROS_INFO("Device found");
  //
  //   /* Try to open the device: requires exclusive access */
  //   res = uvc_open(dev, &devh);
  //
  //   if (res < 0) {
  //     uvc_perror(res, "uvc_open"); /* unable to open device */
  //     ROS_ERROR("Unable to open the device");
  //
  //   } else {
  //     ROS_INFO("Device opened");
  //
  //     uvc_device_descriptor_t *desc;
  //     uvc_get_device_descriptor(dev, &desc);
  //     std::stringstream serial_nr_ss;
  //     serial_nr_ss << desc->idVendor << "_" << desc->idProduct
  //                  << "_TODO_SERIAL_NR";
  //     uvc_free_device_descriptor(desc);
  //     std_msgs::String str_msg;
  //     str_msg.data = serial_nr_ss.str();
  //     serial_nr_pub.publish(str_msg);
  //
  //     /* Try to negotiate a 640x480 30 fps YUYV stream profile */
  //     res = uvc_get_stream_ctrl_format_size(
  //         devh, &ctrl,           /* result stored in ctrl */
  //         UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED
  // */
  //         768, 480, 30           /* width, height, fps */
  //         );
  //
  //     if (res < 0) {
  //       uvc_perror(res,
  //                  "get_mode"); /* device doesn't provide a matching stream
  // */
  //       ROS_ERROR("Device doesn't provide a matching stream");
  //
  //     } else {
  //
  //       /*if (!repeatedStart(devh, ctrl)) {
  //               ROS_ERROR("Failed to get stream from the camera. Try
  //       powercycling the device");
  //               return -1;
  //       }*/
  //
  //       /* Start the video stream. The library will call user function cb:
  //        *   cb(frame, (void*) vio_sensor_pub)
  //        */
  //       res = uvc_start_streaming(devh, &ctrl, uvc_cb, &user_data, 0);
  //       int sleepTime_us = 200000;
  //       usleep(sleepTime_us);
  //       while (!uvc_cb_flag) {
  //         printf("retry start streaming...\n");
  //         uvc_stop_streaming(devh);
  //         res = uvc_start_streaming(devh, &ctrl, uvc_cb, &user_data, 0);
  //         usleep(sleepTime_us);
  //       }
  //
  //       if (res < 0) {
  //         uvc_perror(res, "start_streaming"); /* unable to start stream */
  //         ROS_ERROR("Failed to start stream");
  //
  //       } else {
  //
  //         while (!request_shutdown_flag) {
  //           ros::spinOnce();
  //         }
  //
  //         set_param(sp, "CAMERA_ENABLE", float(0));
  //         ROS_INFO("Wait Sensor Shutdown.");
  //         usleep(1500000);
  //         ROS_INFO("Sensor Shutdown.");
  //         uvc_stop_streaming(devh);
  //         ROS_INFO("Done streaming.");
  //         ros::shutdown();
  //       }
  //     }
  //
  //     /* Release our handle on the device */
  //     sp.close_serial();
  //     uvc_close(devh);
  //     ROS_INFO("Device closed");
  //   }
  //
  //   /* Release the device descriptor */
  //   uvc_unref_device(dev);
  // }
  //
  // /* Close the UVC context. This closes and cleans up any existing device
  //  * handles,
  //  * and it closes the libusb context if one was not provided. */
  // uvc_exit(ctx);
  // ROS_INFO("UVC exited");
  return 0;
}
