#pragma once
#include <vector>
#include <iostream>
#include <cstring>
#include <fstream>
#include "Eigen.h"
#include "Sensor.h"

typedef unsigned char BYTE;

// reads sensor files according to https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

class VirtualSensor: public Sensor
{
public:
	VirtualSensor(std::string directory, int step_size=1);
	bool ProcessNextFrame();
private:
    const std::string m_directory;
    const int m_step_size;

	bool LoadDepthFile(std::string filename);
	bool LoadIntrinsics();

	std::vector<std::string> SplitOnWhitespace(std::string line);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
