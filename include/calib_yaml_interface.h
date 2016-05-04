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
	int cam0_CameraModel;
	int cam0_DistortionModel;

	double cam1_FocalLength[2];
	double cam1_PrincipalPoint[2];
	double cam1_DistortionCoeffs[4];
	int cam1_CameraModel;
	int cam1_DistortionModel;

	double CameraTransformationMatrix[4][4];
	enum {PINHOLE = 0, OMNI = 1};
	enum {RADTAN = 0, EQUI = 1};
};


inline CameraParameters parseYaml(const YAML::Node &node)
{
	CameraParameters v;

	std::string pinhole = "pinhole";
	std::string omni = "omni";

	std::string radtan = "radtan";
	std::string equi = "equi";

	//-------------------------camera models----------------------------
	YAML::Node CameraModel0 = node["cam0"]["camera_model"];

	if (!omni.compare(CameraModel0.as<std::string>())) {
		v.cam0_CameraModel = v.OMNI;

	} else {
		v.cam0_CameraModel = v.PINHOLE;
	}

	YAML::Node CameraModel1 = node["cam1"]["camera_model"];

	if (!omni.compare(CameraModel1.as<std::string>())) {
		v.cam1_CameraModel = v.OMNI;

	} else {
		v.cam1_CameraModel = v.PINHOLE;
	}

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

	if (v.cam0_CameraModel == v.OMNI) {
		v.cam0_FocalLength[0] = intrinsics0[1].as<double>();
		v.cam0_FocalLength[1] = intrinsics0[2].as<double>();
		v.cam0_PrincipalPoint[0] = intrinsics0[3].as<double>();
		v.cam0_PrincipalPoint[1] = intrinsics0[4].as<double>();
	} else {
		v.cam0_FocalLength[0] = intrinsics0[0].as<double>();
		v.cam0_FocalLength[1] = intrinsics0[1].as<double>();
		v.cam0_PrincipalPoint[0] = intrinsics0[2].as<double>();
		v.cam0_PrincipalPoint[1] = intrinsics0[3].as<double>();
	}

	YAML::Node intrinsics1 = node["cam1"]["intrinsics"];
	
	if (v.cam1_CameraModel == v.OMNI) {
		v.cam1_FocalLength[0] = intrinsics1[1].as<double>();
		v.cam1_FocalLength[1] = intrinsics1[2].as<double>();
		v.cam1_PrincipalPoint[0] = intrinsics1[3].as<double>();
		v.cam1_PrincipalPoint[1] = intrinsics0[4].as<double>();
	} else {
		v.cam1_FocalLength[0] = intrinsics1[0].as<double>();
		v.cam1_FocalLength[1] = intrinsics1[1].as<double>();
		v.cam1_PrincipalPoint[0] = intrinsics1[2].as<double>();
		v.cam1_PrincipalPoint[1] = intrinsics1[3].as<double>();
	}

	return v;
}

#endif
