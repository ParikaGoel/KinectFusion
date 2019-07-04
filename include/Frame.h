#pragma once

#include <fstream>
#include <sophus/se3.hpp>

#include <limits>

#include <vector>

#ifndef MINF
#define MINF -std::numeric_limits<double>::infinity()
#endif




class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame(double* depthMap, const Eigen::Matrix3d& depthIntrinsics,
            const unsigned int width, const unsigned int height, int downsampleFactor = 1, double maxDistance = 0.1);

//    Frame(std::vector<double> depthMap, const Eigen::Matrix3d& depthIntrinsics, const Eigen::Matrix4d& depthExtrinsics,
//            const unsigned int width, const unsigned int height);
//
//    Frame(std::vector<double> depthMap, const Eigen::Matrix3d& depthIntrinsics, const Eigen::Matrix4d& depthExtrinsics,
//            const unsigned width, const unsigned height, unsigned downsampleFactor = 1, float maxDistance = 0.1f);

	void computeNormals(double maxDistance=0.1);

	bool readFromFile(const std::string& filename);


    void applyGlobalPose(Sophus::SE3d& estimated_pose);

	const std::vector<Eigen::Vector3d>& getPoints() const;

	const std::vector<Eigen::Vector3d>& getNormals() const;

    const std::vector<Eigen::Vector3d>& getGlobalPoints() const;

    const std::vector<Eigen::Vector3d>& getGlobalNormals() const;

    const Sophus::SE3d& getGlobalPose() const;

    const std::vector<double>& getDepthMap() const;

    const Eigen::Matrix3d& getIntrinsics() const;

    const unsigned int getWidth() const;

    const unsigned int getHeight() const;
    std::vector<double> m_depth_map;

private:

    std::vector<Eigen::Vector3d> computeCameraCoordinates(unsigned int width, unsigned int height);

    std::vector<Eigen::Vector3d> computeNormals(std::vector<double>& depthMap, unsigned int width, unsigned int height, double maxDistance = 0.1);

    void addValidPoints(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals, int downsampleFactor);


    std::vector<Eigen::Vector3d> m_points;
	std::vector<Eigen::Vector3d> m_normals;

    std::vector<Eigen::Vector3d> m_points_global;
    std::vector<Eigen::Vector3d> m_normals_global;
    Sophus::SE3d global_pose;
    Eigen::Matrix3d m_intrinsic_matrix;

    const unsigned int m_width;
    const unsigned int m_height;
};
