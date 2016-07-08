#ifndef STEREOHOMOGRAPHY_H_
#define STEREOHOMOGRAPHY_H_

#include <Eigen/Dense>
#include "fpga_calibration.h"


class StereoHomography
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	StereoHomography(const uvc_ros_driver::FPGACalibration &calib_cam0,
			 const uvc_ros_driver::FPGACalibration &calib_cam1);

	//Computes homography for stereo rectification
	void getHomography(Eigen::Matrix3d &H0, Eigen::Matrix3d &H1, double &f_new, Eigen::Vector2d &p0_new,
			   Eigen::Vector2d &p1_new);

private:
	double r0_[9];
	double t0_[3];
	double f0_[2];
	double p0_[2];
	double d0_[5];
	double r1_[9];
	double t1_[3];
	double f1_[2];
	double p1_[2];
	double d1_[5];
	int image_width_;
	int image_height_;
};

#endif /* STEREOHOMOGRAPHY_H_ */