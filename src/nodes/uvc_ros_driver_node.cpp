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

#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>
#include <sstream>

#include <ros/ros.h>
#include "ait_ros_messages/VioSensorMsg.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include <unistd.h> //debug

static const double acc_scale_factor = 16384.0;
static const double gyr_scale_factor = 131.0;
static const double deg2rad = 2 * M_PI / 360.0;

static double acc_x_prev, acc_y_prev, acc_z_prev, gyr_x_prev, gyr_y_prev, gyr_z_prev;
static ros::Time last_time;

// struct holding all data needed in the callback
struct UserData {
    ros::Publisher pub;
    bool hflip;
};

int16_t ShortSwap(int16_t s) {
    unsigned char b1, b2;

    b1 = s & 255;
    b2 = (s >> 8) & 255;

    return (b1 << 8) + b2;
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void uvc_cb(uvc_frame_t *frame, void *user_ptr) {

    UserData *user_data = (UserData*) user_ptr;

    ait_ros_messages::VioSensorMsg msg;

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

    int frame_size = frame->height * frame->width * 2;

    // read the IMU data
    int16_t zero = 0;
    for (int i = 0; i < frame->height; i += 1) {
        double acc_x = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 0)])) / (acc_scale_factor / 9.81);
        double acc_y = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 1)])) / (acc_scale_factor / 9.81);
        double acc_z = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 2)])) / (acc_scale_factor / 9.81);

        double gyr_x = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 3)]) / (gyr_scale_factor / deg2rad));
        double gyr_y = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 4)]) / (gyr_scale_factor / deg2rad));
        double gyr_z = double(ShortSwap(static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + 5)]) / (gyr_scale_factor / deg2rad));

        if (!(acc_x == acc_x_prev && acc_y == acc_y_prev && acc_z == acc_z_prev && gyr_x == gyr_x_prev && gyr_y == gyr_y_prev && gyr_z == gyr_z_prev)) {
//            printf("\nacc x %+02.4f ", acc_x);
//            printf("acc y %+04.4f ", acc_y);
//            printf("acc z %+04.4f ", acc_z);
//            printf("gyr x %+04.4f ", gyr_x);
//            printf("gyr y %+04.4f ", gyr_y);
//            printf("gyr z %+04.4f\n", gyr_z);

            sensor_msgs::Imu imu_msg;
            imu_msg.linear_acceleration.x = -acc_y;
            imu_msg.linear_acceleration.y = -acc_x;
            imu_msg.linear_acceleration.z = -acc_z;

            imu_msg.angular_velocity.x = -gyr_y;
            imu_msg.angular_velocity.y = -gyr_x;
            imu_msg.angular_velocity.z = -gyr_z;

            msg.imu.push_back(imu_msg);

            acc_x_prev = acc_x;
            acc_y_prev = acc_y;
            acc_z_prev = acc_z;
            gyr_x_prev = gyr_x;
            gyr_y_prev = gyr_y;
            gyr_z_prev = gyr_z;
        } else {
//            printf("skipping same value\n");
        }

        for (int j = 0; j < 6; j++)
            static_cast<int16_t*>(frame->data)[int((i + 1) * frame->width - 6 + j)] = zero;
    }

    // linearly interpolate the time stamps of the imu messages
    ros::Duration elapsed = msg.header.stamp - last_time;
    last_time = msg.header.stamp;

    ros::Time stamp_time = msg.header.stamp;

    printf("%d imu messages\n", msg.imu.size());

    for (int i = 0; i < msg.imu.size(); i++) {
        msg.imu[i].header.stamp = stamp_time - ros::Duration(elapsed*(double(i)/msg.imu.size()));
    }

    // read the image data
    if (user_data->hflip) {
        for (int i = frame_size; i > 0; i -= 2) {
            msg.left_image.data.push_back((static_cast<unsigned char*>(frame->data)[i]));  // left image
            msg.right_image.data.push_back((static_cast<unsigned char*>(frame->data)[i + 1]));  // right image
        }
    } else {
        for (int i = 0; i < frame_size; i += 2) {
            msg.left_image.data.push_back((static_cast<unsigned char*>(frame->data)[i]));  // left image
            msg.right_image.data.push_back((static_cast<unsigned char*>(frame->data)[i + 1]));  // right image
        }
    }

    user_data->pub.publish(msg);
//    user_data->pub.publish(msg.left_image);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "uvc_ros_driver");
    ros::NodeHandle nh("~");  // private nodehandle

    last_time = ros::Time::now();

    UserData user_data;
    user_data.pub = nh.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor", 1);
//    user_data.pub = nh.advertise<sensor_msgs::Image>("/vio_sensor", 1);
    nh.param<bool>("hflip", user_data.hflip, false);

    ros::Publisher serial_nr_pub = nh.advertise<std_msgs::String>("/vio_sensor/device_serial_nr", 1, true);

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
    res = uvc_find_device(ctx, &dev, 0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
        ROS_ERROR("No devices found");
    } else {
        puts("Device found");


        /* Try to open the device: requires exclusive access */
        res = uvc_open(dev, &devh);
        if (res < 0) {
            uvc_perror(res, "uvc_open"); /* unable to open device */
            ROS_ERROR("Unable to open the device");
        } else {
            puts("Device opened");

            /* Print out a message containing all the information that libuvc
             * knows about the device */
//            uvc_print_diag(devh, stderr);

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
            /* Print out the result */
//            uvc_print_stream_ctrl(&ctrl, stderr);
            if (res < 0) {
                uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
                ROS_ERROR("Device doesn't provide a matching stream");
            } else {
                /* Start the video stream. The library will call user function cb:
                 *   cb(frame, (void*) vio_sensor_pub)
                 */
                res = uvc_start_streaming(devh, &ctrl, uvc_cb, &user_data, 0);
                if (res < 0) {
                    uvc_perror(res, "start_streaming"); /* unable to start stream */
                } else {
                    puts("Starting stream... (if images don't start to be published, you may have to powercycle the camera)");
//                    uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */

                    ros::spin();

                    /* End the stream. Blocks until last callback is serviced */
                    uvc_stop_streaming(devh);
                    puts("Done streaming.");
                }
            }
            /* Release our handle on the device */
            uvc_close(devh);
            puts("Device closed");
        }
        /* Release the device descriptor */
        uvc_unref_device(dev);
    }
    /* Close the UVC context. This closes and cleans up any existing device handles,
     * and it closes the libusb context if one was not provided. */
    uvc_exit(ctx);
    puts("UVC exited");
    return 0;
}
