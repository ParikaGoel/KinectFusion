//
// Created by frankzl on 03.07.19.
//
#include "Frame.h"

Frame::Frame(double * depthMap, const Eigen::Matrix3d &depthIntrinsics,
        const unsigned int width, const unsigned int height, int downsampleFactor, double maxDistance)
        : m_width(width), m_height(height),m_intrinsic_matrix(depthIntrinsics){

    m_depth_map.reserve(width*height);

    for (size_t x = 0; x < width*height; x++) {
            m_depth_map.push_back(depthMap[x]);
    }

    auto pointsTmp = computeCameraCoordinates(width, height);
    auto normalsTmp = computeNormals(m_depth_map, width, height, maxDistance);
    addValidPoints(pointsTmp, normalsTmp, downsampleFactor);

}

void Frame::addValidPoints(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals,
                           int downsampleFactor) {

    // We filter out measurements where either point or normal is invalid.
    const unsigned nPoints = points.size();
    m_points.reserve(std::floor(double(nPoints) / downsampleFactor));
    m_normals.reserve(std::floor(double(nPoints) / downsampleFactor));

    for (size_t i = 0; i < nPoints; i = i + downsampleFactor) {
        const auto& point = points[i];
        const auto& normal = normals[i];

        if ( (point.allFinite() && normal.allFinite()) ) {
            m_points.push_back(point);
            m_normals.push_back(normal);
        }else{
            m_depth_map[i] = MINF;
            m_points.emplace_back(Eigen::Vector3d(MINF, MINF, MINF));
            m_normals.emplace_back(Eigen::Vector3d(MINF, MINF, MINF));
        }
    }
}


std::vector<Eigen::Vector3d> Frame::computeCameraCoordinates(unsigned int width, unsigned int height){
    double fovX = m_intrinsic_matrix(0, 0);
    double fovY = m_intrinsic_matrix(1, 1);
    double cX = m_intrinsic_matrix(0, 2);
    double cY = m_intrinsic_matrix(1, 2);

//    auto intrinsicInverse = m_intrinsic_matrix.inverse();


    // Back-project the pixel depths into the camera space.
    std::vector<Eigen::Vector3d> pointsTmp(width * height);

    for (size_t y = 0; y < height; ++y){
        for (size_t x = 0; x < width; ++x){
            unsigned int idx = x + (y * width);
            double depth = m_depth_map[idx];

            if (depth == MINF) {
                pointsTmp[idx] = Eigen::Vector3d(MINF, MINF, MINF);
            }
            else {
                // Back-projection to camera space.
//                Eigen::Vector3d point (x, y, depth);
                  pointsTmp[idx] = Eigen::Vector3d((x - cX) / fovX * depth, (y - cY) / fovY * depth, depth);
//                pointsTmp[idx] = intrinsicInverse * point;
            }
        }
    }
    return pointsTmp;
}


std::vector<Eigen::Vector3d> Frame::computeNormals(std::vector<double>& depthMap, unsigned int width, unsigned int height,  double maxDistance){

    // We need to compute derivatives and then the normalized normal vector (for valid pixels).
    std::vector<Eigen::Vector3d> normalsTmp(width * height);

    for (size_t v = 1; v < height - 1; ++v) {
        for (size_t u = 1; u < width - 1; ++u) {
            unsigned int idx = v*width + u; // linearized index

            const double du = depthMap[idx + 1] - depthMap[idx - 1];
            const double dv = depthMap[idx + width] - depthMap[idx - width];
            if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistance || abs(dv) > maxDistance) {
                normalsTmp[idx] = Eigen::Vector3d(MINF, MINF, MINF);
                continue;
            }

            // TODO: Compute the normals using the cross product of approximate tangent vectors.

            const double xu = 1;
            const double xv = 0;
            const double yu = 0;
            const double yv = 1;

            normalsTmp[idx] = Eigen::Vector3d(yu*dv - yv*du, xv*du - xu*dv, xu*yv - yu*xv);
            normalsTmp[idx].normalize();
        }
    }

    // We set invalid normals for border regions.
    for (size_t u = 0; u < width; ++u) {
        normalsTmp[u] = Eigen::Vector3d(MINF, MINF, MINF);
        normalsTmp[u + (height - 1) * width] = Eigen::Vector3d(MINF, MINF, MINF);
    }
    for (size_t v = 0; v < height; ++v) {
        normalsTmp[v * width] = Eigen::Vector3d(MINF, MINF, MINF);
        normalsTmp[(width - 1) + v * width] = Eigen::Vector3d(MINF, MINF, MINF);
    }
    return normalsTmp;
}

void Frame::applyGlobalPose(Sophus::SE3d& estimated_pose){
    for(auto& point : m_points){
        Eigen::Vector3d g_point = estimated_pose * point;
        m_points_global.emplace_back(g_point);
    }

    for(auto& normal : m_normals){
        Eigen::Vector3d g_normal = estimated_pose.rotationMatrix()*normal;
        m_normals_global.emplace_back(g_normal);
    }
}

const std::vector<Eigen::Vector3d>& Frame::getPoints() const {
        return m_points;
}

const std::vector<Eigen::Vector3d>& Frame::getNormals() const {
        return m_normals;
}

const std::vector<Eigen::Vector3d>& Frame::getGlobalNormals() const{
    return m_normals_global;
}

const std::vector<Eigen::Vector3d>& Frame::getGlobalPoints() const{
        return m_points_global;
}

const Sophus::SE3d& Frame::getGlobalPose() const{
        return global_pose;
}

const std::vector<double>& Frame::getDepthMap() const{
        return m_depth_map;
}

const Eigen::Matrix3d& Frame::getIntrinsics() const{
        return m_intrinsic_matrix;
}

const unsigned int Frame::getWidth() const{
        return m_width;
}

const unsigned int Frame:: getHeight() const{
        return m_height;
}