#ifndef CALIB_YAML_INTERFACE_H_
#define CALIB_YAML_INTERFACE_H_

#include <yaml-cpp/yaml.h>
#include <string>
#include <sstream>
#include "logging.h"

// cameraParameters
// =========================================================
struct CameraParameters { // parameters of one camera
	bool isValid;
	double FocalLength[10][2];
	double PrincipalPoint[10][2];
	double DistortionCoeffs[10][4];
	int CameraModel[10];
	int DistortionModel[10];

	double StereoTransformationMatrix[5][4][4];

	enum {PINHOLE = 0, OMNI = 1};
	enum {RADTAN = 0, EQUI = 1};
};


inline CameraParameters parseYaml(const YAML::Node &node)
{
	CameraParameters v;
	v.isValid = true;

	std::string pinhole = "pinhole";
	std::string omni = "omni";

	std::string radtan = "radtan";
	std::string equi = "equi";

	std::ostringstream cam_name;

	//-------------------------camera models----------------------------
	for (int h = 0; h < int(node.size()); h++) {
		cam_name.str("");
		cam_name.clear();
		cam_name << "cam" << h;
		YAML::Node CameraModel = node[cam_name.str()]["camera_model"];

		if (!omni.compare(CameraModel.as<std::string>())) {
			v.CameraModel[h] = v.OMNI;

		} else {
			v.CameraModel[h] = v.PINHOLE;
		}
	}

	//-------------------------camera transformation matrix----------------------------
	for (int h = 0; h < int(node.size()) / 2; h++) {
		cam_name.str("");
		cam_name.clear();
		cam_name << "cam" << h * 2 + 1;
		YAML::Node CameraTransformationMatrix = node[cam_name.str()]["T_cn_cnm1"];

		for (std::size_t i = 0; i < CameraTransformationMatrix.size(); i++) {
			for (std::size_t j = 0; j < CameraTransformationMatrix[i].size(); j++) {
				v.StereoTransformationMatrix[h][i][j] = CameraTransformationMatrix[i][j].as<double>();
			}
		}
	}

	//-------------------------distortion models----------------------------
	for (int h = 0; h < int(node.size()); h++) {
		cam_name.str("");
		cam_name.clear();
		cam_name << "cam" << h;
		YAML::Node DistortionModel = node[cam_name.str()]["distortion_model"];

		if (!equi.compare(DistortionModel.as<std::string>())) {
			v.DistortionModel[h] = v.EQUI;

		} else {
			v.DistortionModel[h] = v.RADTAN;
		}
	}

	//-------------------------distortion coeffs----------------------------
	for (int h = 0; h < int(node.size()); h++) {
		cam_name.str("");
		cam_name.clear();
		cam_name << "cam" << h;
		YAML::Node RadialDistortion = node[cam_name.str()]["distortion_coeffs"];

		for (std::size_t i = 0; i < RadialDistortion.size(); i++) {
			v.DistortionCoeffs[h][i] = RadialDistortion[i].as<double>();
		}
	}

	//-------------------------focal lengths and principal points-----------
	for (int h = 0; h < int(node.size()); h++) {
		cam_name.str("");
		cam_name.clear();
		cam_name << "cam" << h;
		YAML::Node intrinsics = node[cam_name.str()]["intrinsics"];

		if (v.CameraModel[h] == v.OMNI) {
			v.FocalLength[h][0] = intrinsics[1].as<double>();
			v.FocalLength[h][1] = intrinsics[2].as<double>();
			v.PrincipalPoint[h][0] = intrinsics[3].as<double>();
			v.PrincipalPoint[h][1] = intrinsics[4].as<double>();

		} else {
			v.FocalLength[h][0] = intrinsics[0].as<double>();
			v.FocalLength[h][1] = intrinsics[1].as<double>();
			v.PrincipalPoint[h][0] = intrinsics[2].as<double>();
			v.PrincipalPoint[h][1] = intrinsics[3].as<double>();
		}
	}


	return v;
}

#endif
