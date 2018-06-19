#ifndef FPGA_CALIBRATION_H_
#define FPGA_CALIBRATION_H_

namespace uvc_ros_driver
{

enum ProjectionModelTypes {
	PINHOLE = 0,
	OMNI
};

enum DistortionModelTypes {
	RADTAN = 0,
	EQUI
};

class CameraProjectionModel
{

public:
	CameraProjectionModel() {}

	ProjectionModelTypes projection_type_;
	DistortionModelTypes distortion_type_;

	double focal_length_u_;
	double focal_length_v_;
	double principal_point_u_;
	double principal_point_v_;
	double k1_;
	double k2_;
	double r1_;
	double r2_;
	double t_[3];
	double R_[9];
};

class FPGACalibration
{

public:
	FPGACalibration() {}
	~FPGACalibration() {}

	const CameraProjectionModel *getProjectionModel() const;

	CameraProjectionModel projection_model_;
};
};

#endif /* FPGA_CALIBRATION_H_ */
