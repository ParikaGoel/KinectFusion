#pragma once
#include <algorithm>
#include <fstream>

#include <limits>
#include <cmath>

#include <vector>
#include <Eigen.h>

#ifndef MINF
#define MINF -std::numeric_limits<double>::infinity()
#endif

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Frame(const double* depthMap, const Eigen::Matrix3d& depthIntrinsics,
            const unsigned int width, const unsigned int height, double maxDistance = 2);

	bool WriteMesh(const std::string& filename, std::string color);

    void applyGlobalPose(Eigen::Matrix4d& estimated_pose);

	const std::vector<Eigen::Vector3d>& getPoints() const;

	const std::vector<Eigen::Vector3d>& getNormals() const;

    const std::vector<Eigen::Vector3d>& getGlobalPoints() const;

    const std::vector<Eigen::Vector3d>& getGlobalNormals() const;

    const Eigen::Matrix4d& getGlobalPose() const;

    void setGlobalPose(const Eigen::Matrix4d& pose);

    const std::vector<double>& getDepthMap() const;

    const Eigen::Matrix3d& getIntrinsics() const;

    const unsigned int getWidth() const;

    const unsigned int getHeight() const;

    bool contains(const Eigen::Vector2i& point);

    Eigen::Vector3d projectIntoCamera(const Eigen::Vector3d& globalCoord);
    Eigen::Vector2i projectOntoPlane(const Eigen::Vector3d& cameraCoord);

private:

    std::vector<Eigen::Vector3d> computeCameraCoordinates(unsigned int width, unsigned int height);

    std::vector<Eigen::Vector3d> computeNormals(std::vector<Eigen::Vector3d> camera_points, unsigned int width, unsigned int height, double maxDistance = 0.1);

    void addValidPoints(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals);
    std::vector<Eigen::Vector3d> transformPoints(std::vector<Eigen::Vector3d>& points, Eigen::Matrix4d& transformation);

    std::vector<Eigen::Vector3d> rotatePoints(std::vector<Eigen::Vector3d>& points, Eigen::Matrix3d& rotation);


    std::vector<Eigen::Vector3d> m_points;
	std::vector<Eigen::Vector3d> m_normals;

	const unsigned int m_width;
    const unsigned int m_height;

    std::vector<Eigen::Vector3d> m_points_global;
    std::vector<Eigen::Vector3d> m_normals_global;
    Eigen::Matrix4d m_global_pose;
    Eigen::Matrix3d m_intrinsic_matrix;
    std::vector<double> m_depth_map;
};