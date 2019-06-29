#pragma once

#include <vector>
#include <iostream>
#include <cstring>
#include <fstream>
#include <memory>
#include <EigenHelper.hpp>
#include "FreeImageHelper.hpp"
#include <Utils.hpp>

typedef unsigned char BYTE;

// reads sensor files according to https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
class VirtualSensor
{
public:

	VirtualSensor();

	~VirtualSensor();

	bool Init(const std::string& datasetDir);

	bool ProcessNextFrame();

	unsigned int GetCurrentFrameCnt();

	// get current color data
	BYTE* GetColorRGBX();

	// get current depth data
	std::shared_ptr<std::vector<Point2D>> GetDepth();

	// color camera info
	Eigen::Matrix3d GetColorIntrinsics();

	Eigen::Matrix4d GetColorExtrinsics();

	unsigned int GetColorImageWidth();

	unsigned int GetColorImageHeight();

	// depth (ir) camera info
	Eigen::Matrix3d GetDepthIntrinsics();

	Eigen::Matrix4d GetDepthExtrinsics();

	unsigned int GetDepthImageWidth();

	unsigned int GetDepthImageHeight();

	// get current trajectory transformation
	Eigen::Matrix4d GetTrajectory();

private:

	bool ReadFileList(const std::string& filename, std::vector<std::string>& result, std::vector<double>& timestamps);

	bool ReadTrajectoryFile(const std::string& filename, std::vector<Eigen::Matrix4d>& result, std::vector<double>&
	        timestamps);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// current frame index
	int m_currentIdx;

	int m_increment;

	// frame data
    std::shared_ptr<std::vector<Point2D>>  m_depthFrame;

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
