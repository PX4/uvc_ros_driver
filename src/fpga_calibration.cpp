
#include "fpga_calibration.h"

const uvc_ros_driver::CameraProjectionModel* uvc_ros_driver::FPGACalibration::getProjectionModel() const {
	return &projection_model_;
}
