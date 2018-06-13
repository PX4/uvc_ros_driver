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

inline void setCameraInfoRotation(sensor_msgs::CameraInfo &ci,Eigen::Matrix3d &Rotation)
{
	ci.R[0] = Rotation(0, 0);
	ci.R[1] = Rotation(0, 1);
	ci.R[2] = Rotation(0, 2);
	ci.R[3] = Rotation(1, 0);
	ci.R[4] = Rotation(1, 1);
	ci.R[5] = Rotation(1, 2);
	ci.R[6] = Rotation(2, 0);
	ci.R[7] = Rotation(2, 1);
	ci.R[8] = Rotation(2, 2);
}

inline void setCameraInfoProjection(sensor_msgs::CameraInfo &ci,double fx,
				    double fy, double cx, double cy, double tx, double ty)
{
	ci.P[0] = fx;
	ci.P[1] = 0;
	ci.P[2] = cx;
	ci.P[3] = tx;
	ci.P[4] = 0;
	ci.P[5] = fy;
	ci.P[6] = cy;
	ci.P[7] = ty;
	ci.P[8] = 0;
	ci.P[9] = 0;
	ci.P[10] = 1;
	ci.P[11] = 0;
}


inline void setCameraInfoDistortionMdl(
	sensor_msgs::CameraInfo &ci, uvc_ros_driver::DistortionModelTypes dmt)
{
	std::string model;

/*	if ((int)dmt == 0){
		model = "RADTAN";
	}else{
		model = "EQUIDISTANT";	
	}
*/
	switch (dmt) {
	case uvc_ros_driver::RADTAN :
		model = "plumb_bob";
		break;
	case uvc_ros_driver::EQUI :
	default:
		model = "EQUIDISTANT";	
	}	

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
