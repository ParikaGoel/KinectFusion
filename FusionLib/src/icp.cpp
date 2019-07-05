#include "icp.h"
#include "iterator"

PointToPlaneConstraint::PointToPlaneConstraint(const Eigen::Vector3d& sourcePoint, const Eigen::Vector3d& targetPoint, const Eigen::Vector3d& targetNormal) :
        m_source_point(sourcePoint),
        m_target_point(targetPoint),
        m_target_normal(targetNormal)
{ }

template <typename T>
bool PointToPlaneConstraint::operator()(T const* const sPose, T* sResiduals) const {

    // map inputs
    Eigen::Map<Sophus::SE3<T> const> const pose(sPose);

    Eigen::Matrix<T, 3, 1> transformed_point = pose * m_source_point;

    Eigen::Matrix<T, 3, 1> diff_point = transformed_point - m_target_point;

    sResiduals[0] = diff_point.dot(m_target_normal);

    return true;
}

ceres::CostFunction* PointToPlaneConstraint::create(const Eigen::Vector3d& sourcePoint, const Eigen::Vector3d& targetPoint, const Eigen::Vector3d& targetNormal) {
    return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, Sophus::SE3d::num_parameters>(
            new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal)
    );
}


icp::icp(double dist_thresh, double normal_thresh){
    dist_threshold = dist_thresh;
    normal_threshold = normal_thresh;
}

// Find corresponding points between current frame and previous frame
// Method Used : Projective Point-Plane data association
// Return : vector of pairs of source and target vertex indices
void icp::findCorrespondence(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, std::vector<std::pair<size_t,size_t>>& corresponding_points,Sophus::SE3d& estimated_pose){

    size_t frame_width = curr_frame->getWidth();
    size_t frame_height = curr_frame->getHeight();

    std::vector<double> curr_depth_map = curr_frame->getDepthMap();
    std::vector<double> prev_depth_map = prev_frame->getDepthMap();


    std::vector<Eigen::Vector3d> prev_frame_points = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> prev_frame_normal_map = prev_frame->getGlobalNormals();
    Sophus::SE3d prev_frame_pose = prev_frame->getGlobalPose();
    Eigen::Matrix3d camera_intrinsics = prev_frame->getIntrinsics();

    std::vector<Eigen::Vector3d> curr_frame_vertex_map = curr_frame->getPoints();
    std::vector<Eigen::Vector3d> curr_frame_normal_map = curr_frame->getNormals();



    for(size_t v = 0; v < frame_height; v++){
        for(size_t u = 0; u< frame_width; u++){
            size_t target_idx = (v * frame_width) + u;
            if (curr_depth_map[target_idx] > 0 && prev_depth_map[target_idx] > 0){
                Eigen::Vector3d target_point_camera = prev_frame_pose.inverse() * prev_frame_points[target_idx];
                Eigen::Vector3d target_point_image = camera_intrinsics * target_point_camera;
                target_point_image = target_point_image/target_point_image[2];

                if(target_point_image[0] < frame_width && target_point_image[1] < frame_height){
                    size_t source_idx = (round(target_point_image[1]) * frame_width) + round(target_point_image[0]);
                    if(curr_depth_map[source_idx]<0)
                        continue;
                    Eigen::Vector3d source_point_camera = estimated_pose * curr_frame_vertex_map[source_idx];
                    Eigen::Vector3d source_point_normal = estimated_pose.rotationMatrix() * curr_frame_normal_map[source_idx];

                    if ((source_point_camera - target_point_camera).norm() < dist_threshold){
                        if(abs(source_point_normal.dot(prev_frame_normal_map[target_idx])) > normal_threshold){
                            corresponding_points.push_back(std::make_pair(source_idx,target_idx));
                        }
                    }
                }
            }

        }
    }


}

void icp::prepareConstraints(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, std::vector<std::pair<size_t,size_t>>& corresponding_points, Sophus::SE3d& pose, ceres::Problem& problem) {

    std::vector<Eigen::Vector3d> target_vertex_map = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> target_normal_map = prev_frame->getGlobalNormals();
    std::vector<Eigen::Vector3d> source_vertex_map = curr_frame->getPoints();

    problem.AddParameterBlock(pose.data(),
                              Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);

    for (const auto& match : corresponding_points){
        size_t source_idx = match.first;
        size_t target_idx = match.second;
        const auto& source_point = source_vertex_map[source_idx];
        const auto& target_point = target_vertex_map[target_idx];
        const auto& target_normal = target_normal_map[target_idx];

        if (!source_point.allFinite() && !target_point.allFinite() && !target_normal.allFinite())
            continue;


        ceres::CostFunction* point_to_plane_cost = PointToPlaneConstraint::create(source_point, target_point, target_normal);

        problem.AddResidualBlock(point_to_plane_cost, nullptr,pose.data());
    }
}

void icp::configureSolver(ceres::Solver::Options& options) {
    // Ceres options.
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.use_nonmonotonic_steps = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = 1;
    options.max_num_iterations = 1;
    options.num_threads = 8;
}

void icp::estimatePose(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, size_t m_nIterations,Sophus::SE3d& estimated_pose) {

     for (size_t i = 0; i < m_nIterations; ++i) {
        // Find corresponding points
        std::vector<std::pair<size_t, size_t>> corresponding_points;
        findCorrespondence(prev_frame, curr_frame, corresponding_points,estimated_pose);

        // Prepare constraints

        Sophus::SE3d incremental_pose;

        ceres::Problem problem;
        prepareConstraints(prev_frame, curr_frame, corresponding_points, incremental_pose, problem);

        // Configure options for the solver.
        ceres::Solver::Options options;
        configureSolver(options);

        // Run the solver (for one iteration).
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        //std::cout << summary.FullReport() << std::endl;

        // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
        estimated_pose = incremental_pose * estimated_pose;

        std::cout << "Optimization iteration done." << std::endl;
    }

}