#pragma once

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include "local_parameterization_se3.hpp"

#include "Frame.h"

class PointToPlaneConstraint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointToPlaneConstraint(const Eigen::Vector3d& sourcePoint, const Eigen::Vector3d& targetPoint, const Eigen::Vector3d& targetNormal);

    template <typename T>
    bool operator()(T const* const sPose, T* sResiduals) const;

    static ceres::CostFunction* create(const Eigen::Vector3d& sourcePoint, const Eigen::Vector3d& targetPoint, const Eigen::Vector3d& targetNormal);

protected:
    Eigen::Vector3d m_source_point;
    Eigen::Vector3d m_target_point;
    Eigen::Vector3d m_target_normal;
};

class icp {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    icp(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> current_frame, double dist_threshold, double normal_threshold);
    void estimatePose(const Sophus::SE3d& initial_pose, Sophus::SE3d& estimated_pose, size_t m_nIterations);

private:

    void findCorrespondence(std::vector<std::pair<size_t,size_t>>& corresponding_points);
    void prepareConstraints(std::vector<std::pair<size_t,size_t>>& corresponding_points, Sophus::SE3d& pose, ceres::Problem& problem);
    void configureSolver(ceres::Solver::Options& options);

    std::shared_ptr<Frame> prev_frame;
    std::shared_ptr<Frame> curr_frame;
    double dist_threshold;
    double normal_threshold;

};
