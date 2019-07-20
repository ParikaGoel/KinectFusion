#pragma once

#include <vector>
#include <iostream>
#include <cstring>
#include <fstream>

#include "Eigen.h"
#include "FreeImageHelper.hpp"
#include "Sensor.h"
#include "data_types.h"

// reads sensor files according to https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
class VirtualSensor : public Sensor {
public:

	VirtualSensor() : m_currentIdx(-1), m_increment(1) { }

	~VirtualSensor() {
		SAFE_DELETE_ARRAY(m_depthFrame);
		SAFE_DELETE_ARRAY(m_colorFrame);
	}

	bool init(const std::string& datasetDir, bool intrinsics=true) ;
	bool processNextFrame();

	unsigned int getCurrentFrameCnt();

	// get current color data
	BYTE* getColorRGBX();

	// get current depth data
	double* getDepth();

	// color camera info
	Eigen::Matrix3d getColorIntrinsics();

	Eigen::Matrix4d getColorExtrinsics();

	unsigned int getColorImageWidth();

	unsigned int getColorImageHeight();

	// depth (ir) camera info
	Eigen::Matrix3d getDepthIntrinsics();

	Eigen::Matrix4d getDepthExtrinsics();

	unsigned int getDepthImageWidth();

	unsigned int getDepthImageHeight();

protected:
	bool readFileList(const std::string& filename, std::vector<std::string>& result, std::vector<double>& timestamps);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// current frame index
	int m_currentIdx;

	int m_increment;

	// frame data
	double* m_depthFrame;
	BYTE* m_colorFrame;
	// color camera info
	Eigen::Matrix4d m_colorExtrinsics;

	// depth (ir) camera info
	Eigen::Matrix4d m_depthExtrinsics;

	// base dir
	std::string m_baseDir;
	// filenamelist depth
	std::vector<std::string> m_filenameDepthImages;
	std::vector<double> m_depthImagesTimeStamps;
	// filenamelist color
	std::vector<std::string> m_filenameColorImages;
	std::vector<double> m_colorImagesTimeStamps;
};