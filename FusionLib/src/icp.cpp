#include "MeshWriter.h"
#include "icp.h"
#include "iterator"

PointToPlaneConstraint::PointToPlaneConstraint(const Eigen::Vector3d& sourcePoint, const Eigen::Vector3d& targetPoint, const Eigen::Vector3d& targetNormal) :
        m_source_point(sourcePoint),
        m_target_point(targetPoint),
        m_target_normal(targetNormal)
{ }

// params: arg to optimize, residuals
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

bool icp::hasValidDistance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
    return (point1- point2).norm() < dist_threshold;
}

bool icp::hasValidAngle(const Eigen::Vector3d& normal1, const Eigen::Vector3d& normal2) {
    return std::abs(normal1.dot(normal2)) > normal_threshold;
}

// Find corresponding points between current frame and previous frame
// Method Used : Projective Point-Plane data association
// Return : vector of pairs of source and target vertex indices
void icp::findCorrespondence(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, std::vector<std::pair<size_t,size_t>>& corresponding_points,Eigen::Matrix4d& estimated_pose){

    std::vector<Eigen::Vector3d> prev_frame_global_points = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> prev_frame_global_normals = prev_frame->getGlobalNormals();

    std::vector<Eigen::Vector3d> curr_frame_points = curr_frame->getPoints();
    std::vector<Eigen::Vector3d> curr_frame_normals = curr_frame->getNormals();

    const auto rotation = estimated_pose.block(0, 0, 3, 3);
    const auto translation = estimated_pose.block(0, 3, 3, 1);

    for(size_t idx = 0; idx < curr_frame_points.size(); idx++){

        Eigen::Vector3d curr_point = curr_frame_points[idx];
        Eigen::Vector3d curr_normal = curr_frame_normals[idx];

        if (curr_point.allFinite() && curr_normal.allFinite()) {
            const Eigen::Vector3d curr_global_point = rotation * curr_point + translation;
            const Eigen::Vector3d curr_global_normal = rotation * curr_normal;

            const Eigen::Vector3d curr_point_prev_frame = prev_frame->projectIntoCamera(curr_global_point);
            const Eigen::Vector2d curr_point_img_coord = prev_frame->projectOntoPlane(curr_point_prev_frame);

            if (prev_frame->contains(curr_point_img_coord)) {

                size_t prev_idx = curr_point_img_coord[1] * prev_frame->getWidth() + curr_point_img_coord[0];

                Eigen::Vector3d prev_global_point = prev_frame_global_points[prev_idx];
                Eigen::Vector3d prev_global_normal = prev_frame_global_normals[prev_idx];

                if (prev_global_point.allFinite() && prev_global_normal.allFinite()) {

                    if(hasValidDistance(prev_global_point, curr_global_point) &&
                       hasValidAngle(prev_global_normal, curr_global_normal)) {

                        corresponding_points.push_back(std::make_pair(prev_idx, idx));

                    }
                }
            }
        }
    }
}

const Eigen::Matrix4d icp::getPose(Eigen::Matrix<double, 6, 1>& x){
    double alpha = x(0);
    double beta  = x(1);
    double gamma = x(2);
    double t_x = x(3);
    double t_y = x(4);
    double t_z = x(5);

    Eigen::Matrix4d pose;
    pose(0,0) = 1;
    pose(0,1) = alpha * beta - gamma;
    pose(0,2) = alpha * gamma + beta;
    pose(0,3) = t_x;

    pose(1,0) = gamma;
    pose(1,1) = alpha * beta * gamma + 1;
    pose(1,2) = beta * gamma - alpha;
    pose(1,3) = t_y;

    pose(2,0) = -beta;
    pose(2,1) = alpha;
    pose(2,2) = 1;
    pose(2,3) = t_z;

    pose(3,0) = 0;
    pose(3,1) = 0;
    pose(3,2) = 0;
    pose(3,3) = 1;


    //pose <<   1   , -gamma,   beta, t_x,
    //        gamma ,    1  , -alpha, t_y,
    //        -beta , alpha ,    1  , t_z,
    //        0   ,    0  ,    0  ,   1;

    return pose;
}

double icp::getb_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i, Eigen::Vector3d& d_i){
    return n_i.dot(d_i) - n_i.dot(s_i);
}

Eigen::Matrix<double, 6, 1> icp::getA_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i){
    // double a_i1 = n_i.z() * s_i.y()  - n_i.y() * s_i.z();
    // double a_i2 = n_i.x() * s_i.z()  - n_i.z() * s_i.x();
    // double a_i3 = n_i.y() * s_i.x()  - n_i.x() * s_i.y();

    // Eigen::Matrix<double, 6, 1> row;
    // row << ( a_i1, a_i2, a_i3, n_i.x(), n_i.y(), n_i.z() );

    Eigen::Matrix<double, 6, 1> row;
    // TODO check
    row << s_i.cross(n_i) , n_i;

    return row;
}




Eigen::Matrix4d icp::solveForPose(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame,
                                  std::vector<std::pair<size_t,size_t>>& corresponding_points){

    std::vector<Eigen::Vector3d> prev_global_points  = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> prev_global_normals = prev_frame->getGlobalNormals();
    std::vector<Eigen::Vector3d> curr_points         = curr_frame->getPoints();

    const size_t N = corresponding_points.size();

    Eigen::MatrixXd A( N , 6);
    Eigen::MatrixXd b( N , 1);

    //for (const auto &match : corresponding_points){
    for (size_t i = 0; i < corresponding_points.size(); ++i ){

        auto match = corresponding_points[i];

        Eigen::Vector3d d_i = prev_global_points[ match.first ];
        Eigen::Vector3d n_i = prev_global_normals[ match.first ];
        Eigen::Vector3d s_i = curr_points[ match.second ];

        A.row(i) = getA_i(s_i, n_i);
        b(i) = getb_i(s_i, n_i, d_i);
    }
    // Eigen::Matrix<double, 6,1> x = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<double, 6,1> x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    return getPose(x);
}



/*
void icp::prepareConstraints(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, std::vector<std::pair<size_t,size_t>>& corresponding_points, Sophus::SE3d& pose, ceres::Problem& problem) {

    std::vector<Eigen::Vector3d> target_vertex_map = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> target_normal_map = prev_frame->getGlobalNormals();
    std::vector<Eigen::Vector3d> source_vertex_map = curr_frame->getPoints();

    // problem.AddParameterBlock(pose.data(),
    //                           Sophus::SE3d::num_parameters,
    //                           new Sophus::test::LocalParameterizationSE3);

    for (const auto& match : corresponding_points){
        size_t source_idx = match.first;
        size_t target_idx = match.second;
        const auto& source_point = source_vertex_map[source_idx];
        const auto& target_point = target_vertex_map[target_idx];
        const auto& target_normal = target_normal_map[target_idx];

        if (!source_point.allFinite() && !target_point.allFinite() && !target_normal.allFinite())
            continue;


        ceres::CostFunction* point_to_plane_cost = PointToPlaneConstraint::create(source_point, target_point, target_normal);

        // params: cost func, loss function (null in our case), argument to optimize)
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

        // findCorrespondence(prev_frame, curr_frame, corresponding_points,estimated_pose);
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

        std::cout << estimated_pose.matrix3x4()<< std::endl;

        std::cout << incremental_pose.matrix3x4()<< std::endl;

        // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
        estimated_pose = incremental_pose * estimated_pose;

        std::cout << estimated_pose.matrix3x4()<< std::endl;

        std::cout << "Optimization iteration done." << std::endl;
    }

}*/

void icp::estimatePose(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, size_t m_nIterations,Eigen::Matrix4d& estimated_pose)
{
    for (size_t i = 0; i < m_nIterations; ++i) {

        std::vector<std::pair<size_t, size_t>> corresponding_points;
        findCorrespondence(prev_frame, curr_frame, corresponding_points, estimated_pose);

        size_t N = corresponding_points.size();
        // N = 200000;

        std::vector<Eigen::Vector3d> matchA (N);
        std::vector<Eigen::Vector3d> matchB (N);


        for (size_t idx = 200000; idx < N; idx++){
            matchA.push_back((prev_frame->getPoints())[corresponding_points[idx].first]);
            matchB.push_back((curr_frame->getPoints())[corresponding_points[idx].second]);
        }

        std::cout << "writing" << MeshWriter::toFile(
                "corrA" + std::to_string(i), "255 0 0 255", matchA);
        std::cout << "writing" << MeshWriter::toFile(
                "corrB" + std::to_string(i), "255 255 0 255", matchB);

        MeshWriter::toFile(
                "meshA", "255 255 255 255", prev_frame->getPoints()
                );

        MeshWriter::toFile(
                "meshB", "255 255 0 255", curr_frame->getPoints()
                );

        Eigen::Matrix4d pose = solveForPose(prev_frame, curr_frame, corresponding_points);

        estimated_pose = pose;
    }
}
