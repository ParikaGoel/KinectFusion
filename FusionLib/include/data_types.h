#pragma once

#include <fstream>
#include "Eigen.h"
#include <iostream>

typedef unsigned char BYTE;

struct Voxel {
	double tsdf;
	double weight;
	Vector4uc color;

	Voxel()
			: tsdf(0.0f), weight(0.0f), color(0, 0, 0, 0) {}
};

struct Config {

public:
	Config(const double dist_threshold, const double normal_threshold, const double truncationDistance,
		   const Eigen::Vector3d volumeOrigin, const int x, const int y, const int z, const double voxelScale)
			:
			m_dist_threshold(dist_threshold)
			, m_normal_threshold(normal_threshold)
			, m_truncationDistance(truncationDistance)
			, m_voxelScale(voxelScale)
			, m_volumeSize(x, y, z)
			, m_volumeOrigin(volumeOrigin) {};

	const double m_dist_threshold;
	const double m_normal_threshold;
	const double m_truncationDistance;
	const double m_voxelScale;
	Eigen::Vector3i m_volumeSize;
	const Eigen::Vector3d m_volumeOrigin;

	std::string toString() {
		std::stringstream ss;
		ss << "Distance Threshold: " << m_dist_threshold << std::endl;
		ss << "Normal Threshold: " << m_normal_threshold << std::endl;
		ss << "Truncation Distance: " << m_truncationDistance << std::endl;
		ss << "Voxel Scale: " << m_voxelScale << std::endl;
		ss << "Volume Size: " << m_volumeSize.transpose() << std::endl;
		ss << "Volume Origin: " << m_volumeOrigin.transpose() << std::endl;

		return ss.str();
	}

	bool printToFile(std::string filename) {
		std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

		// Write off file.
		std::cout << filename << std::endl;
		std::ofstream outFile(filenameBaseOut + filename + ".txt");
		if (!outFile.is_open()) { return false; }
		outFile << toString();
		outFile.close();
		return true;

	}

};
