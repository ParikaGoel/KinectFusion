#pragma once

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include "local_parameterization_se3.hpp"

#include "Frame.h"

class LinearSolver{
public:
    LinearSolver(){}

    void solvePoint2Plane(const std::vector<Eigen::Vector3d>& sourcePoints,
            const std::vector<Eigen::Vector3d>& destPoints,
            const std::vector<Eigen::Vector3d> destNormals,
            const std::vector<std::pair<size_t, size_t>>& correspondence);
    void solvePoint2Point(const std::vector<Eigen::Vector3d>& sourcePoints,
            const std::vector<Eigen::Vector3d>& destPoints,
            const std::vector<std::pair<size_t, size_t>>& correspondence);

    const Eigen::Matrix4d getPose();


private:
    Eigen::Matrix<double, 6, 1> solution;
};


class icp {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    icp(double dist_threshold, double normal_threshold, unsigned int neighbor_range);
    void estimatePose(int i, std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> current_frame, size_t m_nIterations,Eigen::Matrix4d& estimated_pose);

    const Eigen::Matrix4d getPose(Eigen::Matrix<double, 6, 1>& x);

private:

    double getb_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i, Eigen::Vector3d& d_i);
    Eigen::Matrix<double, 6, 1> getA_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i);

    Eigen::Matrix4d solveForPose(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame,
            Eigen::Matrix4d& estimated_pose,
            std::vector<std::pair<size_t,size_t>>& corresponding_points);

    bool hasValidDistance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
    bool hasValidAngle(const Eigen::Vector3d& normal1, const Eigen::Vector3d& normal2);

    void findCorrespondence(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame,std::vector<std::pair<size_t,size_t>>& corresponding_points,Eigen::Matrix4d& estimated_pose);
    void findDistanceCorrespondence(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame,std::vector<std::pair<size_t,size_t>>& corresponding_points,Eigen::Matrix4d& estimated_pose);

    double dist_threshold;
    double normal_threshold;
    const unsigned int neighbor_range;
};