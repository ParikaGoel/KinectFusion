#pragma once

#include <vector>
#include <iostream>
#include <cstring>
#include <fstream>

#include "Eigen.h"
#include "FreeImageHelper.hpp"

typedef unsigned char BYTE;

// reads sensor files according to https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
class VirtualSensor {
public:

	VirtualSensor() : m_currentIdx(-1), m_increment(1) { }

	~VirtualSensor() {
		SAFE_DELETE_ARRAY(m_depthFrame);
		SAFE_DELETE_ARRAY(m_colorFrame);
	}

	bool init(const std::string& datasetDir) ;
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

	// get current trajectory transformation
	Eigen::Matrix4d getTrajectory();

private:
	bool readFileList(const std::string& filename, std::vector<std::string>& result, std::vector<double>& timestamps);

	bool readTrajectoryFile(const std::string& filename, std::vector<Eigen::Matrix4d>& result,
							std::vector<double>& timestamps);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// current frame index
	int m_currentIdx;

	int m_increment;

	// frame data
	double* m_depthFrame;
	BYTE* m_colorFrame;
	Eigen::Matrix4d m_currentTrajectory;

	// color camera info
	Eigen::Matrix3d m_colorIntrinsics;
	Eigen::Matrix4d m_colorExtrinsics;
	unsigned int m_colorImageWidth;
	unsigned int m_colorImageHeight;

	// depth (ir) camera info
	Eigen::Matrix3d m_depthIntrinsics;
	Eigen::Matrix4d m_depthExtrinsics;
	unsigned int m_depthImageWidth;
	unsigned int m_depthImageHeight;

	// base dir
	std::string m_baseDir;
	// filenamelist depth
	std::vector<std::string> m_filenameDepthImages;
	std::vector<double> m_depthImagesTimeStamps;
	// filenamelist color
	std::vector<std::string> m_filenameColorImages;
	std::vector<double> m_colorImagesTimeStamps;

	// trajectory
	std::vector<Eigen::Matrix4d> m_trajectory;
	std::vector<double> m_trajectoryTimeStamps;
};