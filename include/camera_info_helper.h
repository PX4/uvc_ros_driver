#ifndef __CAMERA_INFO_HELPER_H__
#define __CAMERA_INFO_HELPER_H__

#include "fpga_calibration.h"
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <string>

inline void setCameraInfoIntrinsics(sensor_msgs::CameraInfo &ci, double fx,
				    double fy, double cx, double cy)
{
	ci.K[0] = fx;
	ci.K[1] = 0;
	ci.K[2] = cx;
	ci.K[3] = 0;
	ci.K[4] = fy;
	ci.K[5] = cy;
	ci.K[6] = 0;
	ci.K[7] = 0;
	ci.K[8] = 1;
}

inline void setCameraInfoDistortionMdl(
	sensor_msgs::CameraInfo &ci, uvc_ros_driver::ProjectionModelTypes pmt)
{
	std::string model;

	switch (pmt) {
	case uvc_ros_driver::RADTAN:
		model = "RADTAN";
		break;
	case uvc_ros_driver::EQUI:
	default:
		model = "EQUIDISTANT";	
	}	
/*	case uvc_ros_driver::OMNI:
		model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
		break;

	case uvc_ros_driver::PINHOLE:
	default:
		model = sensor_msgs::distortion_models::PLUMB_BOB;
	}
*/
	ci.distortion_model = model;
}

inline void setCameraInfoDistortionParams(sensor_msgs::CameraInfo &ci,
		double k1, double k2, double t1,
		double t2, double k3)
{
	// define size of the distortion vector
	ci.D.resize(5);
	ci.D[0] = k1;
	ci.D[1] = k2;
	ci.D[2] = t1;
	ci.D[3] = t2;
	ci.D[4] = k3;
}

inline void setCameraInfoHeader(sensor_msgs::CameraInfo &ci, int width,
				int height, const ros::Time &t,
				const std::string &frame_id)
{
	ci.width = width;
	ci.height = height;
	ci.header.stamp = t;
	ci.header.frame_id = frame_id;
}

#endif /* end of include guard: __CAMERA_INFO_HELPER_H__ */
