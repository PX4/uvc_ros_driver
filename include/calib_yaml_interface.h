#ifndef CALIB_YAML_INTERFACE_H_
#define CALIB_YAML_INTERFACE_H_

#include <yaml-cpp/yaml.h>
#include <string>
#include "logging.h"

// cameraParameters
// =========================================================
struct CameraParameters { // parameters of one camera
	double cam0_FocalLength[2];
	double cam0_PrincipalPoint[2];
	double cam0_DistortionCoeffs[4];
	int cam0_DistortionModel;

	double cam1_FocalLength[2];
	double cam1_PrincipalPoint[2];
	double cam1_DistortionCoeffs[4];
	int cam1_DistortionModel;

	double CameraTransformationMatrix[4][4];
	enum {RADTAN = 0, EQUI = 1};
};


inline CameraParameters parseYaml(const YAML::Node &node)
{
	CameraParameters v;

	std::string radtan = "radtan";
	std::string equi = "equi";

	//-------------------------camera transformation matrix----------------------------
	YAML::Node CameraTransformationMatrix = node["cam1"]["T_cn_cnm1"];

	for (std::size_t i = 0; i < CameraTransformationMatrix.size(); i++) {
		for (std::size_t j = 0; j < CameraTransformationMatrix[i].size(); j++) {
			v.CameraTransformationMatrix[i][j] = CameraTransformationMatrix[i][j].as<double>();
		}
	}

	//-------------------------distortion models----------------------------
	YAML::Node DistortionModel0 = node["cam0"]["distortion_model"];

	if (!equi.compare(DistortionModel0.as<std::string>())) {
		v.cam0_DistortionModel = v.EQUI;

	} else {
		v.cam0_DistortionModel = v.RADTAN;
	}

	YAML::Node DistortionModel1 = node["cam1"]["distortion_model"];

	if (!equi.compare(DistortionModel1.as<std::string>())) {
		v.cam1_DistortionModel = v.EQUI;

	} else {
		v.cam1_DistortionModel = v.RADTAN;
	}


	//-------------------------distortion coeffs----------------------------
	YAML::Node RadialDistortion0 = node["cam0"]["distortion_coeffs"];

	for (std::size_t i = 0; i < RadialDistortion0.size(); i++) {
		v.cam0_DistortionCoeffs[i] = RadialDistortion0[i].as<double>();
	}

	YAML::Node RadialDistortion1 = node["cam1"]["distortion_coeffs"];

	for (std::size_t i = 0; i < RadialDistortion0.size(); i++) {
		v.cam1_DistortionCoeffs[i] = RadialDistortion1[i].as<double>();
	}

	//-------------------------focal lengths and principal points-----------
	YAML::Node intrinsics0 = node["cam0"]["intrinsics"];
	YAML::Node intrinsics1 = node["cam1"]["intrinsics"];

	for (std::size_t i = 0; i < 2; i++) {
		v.cam0_FocalLength[i] = intrinsics0[i].as<double>();
		v.cam1_FocalLength[i] = intrinsics1[i].as<double>();
	}

	for (std::size_t i = 2; i < intrinsics0.size(); i++) {
		v.cam0_PrincipalPoint[i-2] = intrinsics0[i].as<double>();
		v.cam1_PrincipalPoint[i-2] = intrinsics1[i].as<double>();
	}

	return v;
}

#endif
