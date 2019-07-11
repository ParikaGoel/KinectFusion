#include "MeshWriter.h"
#include "icp.h"
#include "iterator"


void LinearSolver::solvePoint2Plane(const std::vector<Eigen::Vector3d>& sourcePoints,
            const std::vector<Eigen::Vector3d>& destPoints,
            const std::vector<Eigen::Vector3d> destNormals,
            const std::vector<std::pair<size_t, size_t>>& correspondence){

    const size_t N = correspondence.size();
    Eigen::MatrixXd A( N , 6);
    Eigen::MatrixXd b( N , 1);

    for (size_t i = 0; i < correspondence.size(); ++i ){
        auto match = correspondence[i];

        Eigen::Vector3d d_i = destPoints[ match.first ];
        Eigen::Vector3d n_i = destNormals[ match.first ];
        Eigen::Vector3d s_i = sourcePoints[ match.second ];

        Eigen::Matrix<double, 6, 1> A_i;
        A_i << s_i.cross(n_i) , n_i;
        A.row(i) = A_i;
        b(i) = n_i.dot(d_i) - n_i.dot(s_i);
    }
    // Eigen::Matrix<double, 6,1> x = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<double, 6,1> x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    solution = x;
}

void LinearSolver::solvePoint2Point(const std::vector<Eigen::Vector3d>& sourcePoints,
            const std::vector<Eigen::Vector3d>& destPoints,
            const std::vector<std::pair<size_t, size_t>>& correspondence){
    const size_t N = correspondence.size();
    Eigen::MatrixXd A( N*3 , 6);
    Eigen::MatrixXd b( N*3 , 1);
    for (size_t i = 0; i < correspondence.size(); ++i ){
        auto match = correspondence[i];

        Eigen::Vector3d d_i = destPoints[ match.first ];
        Eigen::Vector3d s_i = sourcePoints[ match.second ];

        Eigen::Matrix<double, 3, 6> A_i;
        A_i << 0,    s_i.z(), -s_i.y(), 1, 0, 0,
           -s_i.z(),   0,      s_i.x(), 0, 1, 0,
            s_i.y(), -s_i.x(), 0, 0, 0, 1;

        A.row(i*3) = A_i.row(0);
        A.row(i*3+1) = A_i.row(1);
        A.row(i*3+2) = A_i.row(2);

        b(i*3)     = d_i.x() - s_i.x();
        b(i*3 + 1) = d_i.y() - s_i.y();
        b(i*3 + 2) = d_i.z() - s_i.z();
    }
    // Eigen::Matrix<double, 6,1> x = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<double, 6,1> x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    solution = x;
}

const Eigen::Matrix4d LinearSolver::getPose(){
    Eigen::Matrix4d transformation;
    double alpha = solution[0];
    double beta  = solution[1];
    double gamma = solution[2];

    transformation << 1,     alpha*beta - gamma,   alpha*gamma + beta,     solution[3],
            gamma, alpha*beta*gamma + 1, beta*gamma - alpha,     solution[4],
            -beta,    alpha,                  1,                 solution[5],
            0 , 0, 0, 1;
    return transformation;
}

icp::icp(double dist_thresh, double normal_thresh, unsigned int neighbor_range)
    :dist_threshold(dist_thresh), normal_threshold(normal_thresh), neighbor_range(neighbor_range)
{}

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
            const Eigen::Vector2i curr_point_img_coord = prev_frame->projectOntoPlane(curr_point_prev_frame);

            const Eigen::Vector2i closest_img_coord = prev_frame->findClosestPoint( curr_point_img_coord[0], curr_point_img_coord[1], curr_point_prev_frame, neighbor_range);

            if (prev_frame->contains(closest_img_coord)) {

                // size_t prev_idx = curr_point_img_coord[1] * prev_frame->getWidth() + curr_point_img_coord[0];
                size_t prev_idx = closest_img_coord[1] * prev_frame->getWidth() + closest_img_coord[0];

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

void icp::findDistanceCorrespondence(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, std::vector<std::pair<size_t,size_t>>& corresponding_points,Eigen::Matrix4d& estimated_pose){

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
            const Eigen::Vector2i curr_point_img_coord = prev_frame->projectOntoPlane(curr_point_prev_frame);

            const Eigen::Vector2i closest_img_coord = prev_frame->findClosestDistancePoint( curr_point_img_coord[0], curr_point_img_coord[1], curr_point_prev_frame, neighbor_range);

            if (prev_frame->contains(closest_img_coord)) {

                size_t prev_idx = closest_img_coord[1] * prev_frame->getWidth() + closest_img_coord[0];

                Eigen::Vector3d prev_global_point = prev_frame_global_points[prev_idx];
                Eigen::Vector3d prev_global_normal = prev_frame_global_normals[prev_idx];

                if (prev_global_point.allFinite() ) {
                    if(hasValidDistance(prev_global_point, curr_global_point)) {
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

    std::cout << "estimated trans:"<< pose << std::endl;

    return pose;
}

double icp::getb_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i, Eigen::Vector3d& d_i){
    return n_i.dot(d_i) - n_i.dot(s_i);
}

Eigen::Matrix<double, 6, 1> icp::getA_i(Eigen::Vector3d& s_i, Eigen::Vector3d& n_i){
    double a_i1 = n_i.z() * s_i.y()  - n_i.y() * s_i.z();
    double a_i2 = n_i.x() * s_i.z()  - n_i.z() * s_i.x();
    double a_i3 = n_i.y() * s_i.x()  - n_i.x() * s_i.y();

    Eigen::Matrix<double, 6, 1> row;
    row << a_i1, a_i2, a_i3, n_i;

    // Eigen::Matrix<double, 6, 1> row;
    // // TODO check

    // row << s_i.cross(n_i) , n_i;
    // std::cout << row << std::endl;

    return row;
}




Eigen::Matrix4d icp::solveForPose(std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame,
                                  Eigen::Matrix4d& estimated_pose,
                                  std::vector<std::pair<size_t,size_t>>& corresponding_points){

    std::vector<Eigen::Vector3d> prev_global_points  = prev_frame->getGlobalPoints();
    std::vector<Eigen::Vector3d> prev_global_normals = prev_frame->getGlobalNormals();
    std::vector<Eigen::Vector3d> curr_points         = curr_frame->getPoints();

    const size_t N = corresponding_points.size();

    Eigen::MatrixXd A( N , 6);
    Eigen::MatrixXd b( N , 1);

    const auto rotation = estimated_pose.block(0, 0, 3, 3);
    const auto translation = estimated_pose.block(0, 3, 3, 1);

    //for (const auto &match : corresponding_points){
    for (size_t i = 0; i < corresponding_points.size(); ++i ){

        auto match = corresponding_points[i];

        Eigen::Vector3d d_i = prev_global_points[ match.first ];
        Eigen::Vector3d n_i = prev_global_normals[ match.first ];
        Eigen::Vector3d s_i = rotation * curr_points[ match.second ] + translation;

        A.row(i) = getA_i(s_i, n_i);
        b(i) = getb_i(s_i, n_i, d_i);
    }
    // Eigen::Matrix<double, 6,1> x = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<double, 6,1> x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    return getPose(x);
}

void icp::estimatePose(int frame_cnt, std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, size_t m_nIterations,Eigen::Matrix4d& estimated_pose)
{
    MeshWriter::toFile("meshA" + std::to_string(frame_cnt), "0 255 0 255", prev_frame->getGlobalPoints());
    MeshWriter::toFile("meshB" + std::to_string(frame_cnt), "255 0 0 255", curr_frame->getGlobalPoints());

    for (size_t i = 0; i < m_nIterations; ++i) {

        std::vector<std::pair<size_t, size_t>> corresponding_points;
        findCorrespondence(prev_frame, curr_frame, corresponding_points, estimated_pose);

        size_t N = corresponding_points.size();

        size_t num_splits = 4;

        std::vector<std::vector<Eigen::Vector3d>> splitsA (num_splits);
        std::vector<std::vector<Eigen::Vector3d>> splitsB (num_splits);

        size_t interval_size = N / num_splits;

        std::cout << N;

        // for (size_t q = 0; q < num_splits; q++){

        //     splitsA[q] = std::vector<Eigen::Vector3d>(interval_size);
        //     splitsB[q] = std::vector<Eigen::Vector3d>(interval_size);

        //     for (size_t idx = interval_size * q; idx < interval_size*(q + 1); idx++){
        //         splitsA[q][idx - interval_size * q ] = ((prev_frame->getGlobalPoints())[corresponding_points[idx].first]);
        //         splitsB[q][idx - interval_size * q ] = ((curr_frame->getGlobalPoints())[corresponding_points[idx].second]);
        //     }
        // }

        LinearSolver solver;
        // solver.solvePoint2Point( curr_frame->getGlobalPoints(), prev_frame->getGlobalPoints(), corresponding_points);
        solver.solvePoint2Plane(curr_frame->getGlobalPoints(), prev_frame->getGlobalPoints(),
                                prev_frame->getGlobalNormals(),
                                corresponding_points);

        estimated_pose = solver.getPose() * estimated_pose;


        // std::vector<std::string> colors (corresponding_points.size());
        // colors[0] = "255 0 255 255";
        // colors[1] = "0 0 255 255";
        // colors[2] = "255 0 0 255";
        // colors[3] = "255 255 0 255";

        // for (size_t split_no = 0; split_no < num_splits; split_no++){
        //     MeshWriter::toFile(
        //             "corrB" + std::to_string(i) + "_" + std::to_string(split_no), colors[split_no], splitsB[split_no]);
        //     MeshWriter::toFile(
        //             "corrA" + std::to_string(i) + "_" + std::to_string(split_no), colors[split_no], splitsA[split_no]);
        // }

        //Eigen::Matrix4d pose = solveForPose(prev_frame, curr_frame, estimated_pose, corresponding_points);

        //estimated_pose = pose * estimated_pose;

        curr_frame->setGlobalPose(estimated_pose);
    }
    MeshWriter::toFile(
                "corrBT" + std::to_string(frame_cnt), "0 0 255 255", curr_frame->getGlobalPoints());
}
