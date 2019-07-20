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

icp::icp(double dist_thresh, double normal_thresh)
    :dist_threshold(dist_thresh), normal_threshold(normal_thresh)
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
            const Eigen::Vector2i curr_point_img_coord = prev_frame->projectOntoDepthPlane(curr_point_prev_frame);

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

bool icp::estimatePose(int frame_cnt, std::shared_ptr<Frame> prev_frame, std::shared_ptr<Frame> curr_frame, size_t m_nIterations,Eigen::Matrix4d& estimated_pose)
{
//    MeshWriter::toFile("meshA" + std::to_string(frame_cnt), "0 255 0 255", prev_frame->getGlobalPoints());
//    MeshWriter::toFile("meshB" + std::to_string(frame_cnt), "255 0 0 255", curr_frame->getGlobalPoints());

    for (size_t i = 0; i < m_nIterations; ++i) {

        std::vector<std::pair<size_t, size_t>> corresponding_points;
        findCorrespondence(prev_frame, curr_frame, corresponding_points, estimated_pose);

        LinearSolver solver;
        solver.solvePoint2Plane(curr_frame->getGlobalPoints(), prev_frame->getGlobalPoints(),
                                prev_frame->getGlobalNormals(),
                                corresponding_points);

        estimated_pose = solver.getPose() * estimated_pose;

        curr_frame->setGlobalPose(estimated_pose);
    }
//    MeshWriter::toFile(
//                "corrBT" + std::to_string(frame_cnt), "0 0 255 255", curr_frame->getGlobalPoints());

    return true;
}
