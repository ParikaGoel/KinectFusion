#include "Frame.h"

#include <iostream>

Frame::Frame(double * depthMap, const Eigen::Matrix3d &depthIntrinsics,
        const unsigned int width, const unsigned int height, int downsampleFactor, double maxDistance)
        : m_height(height),m_width(width),m_intrinsic_matrix(depthIntrinsics),_rawDepthMap(depthMap){

    m_depth_map.reserve(width*height);

    std::cout << std::endl;
    for (size_t x = 0; x < width*height; x++) {
            m_depth_map.push_back(depthMap[x]);
    }

    auto pointsTmp = computeCameraCoordinates(width, height);
    auto normalsTmp = computeNormals(pointsTmp, width, height, maxDistance);

    addValidPoints(pointsTmp, normalsTmp);

    setGlobalPose(Eigen::Matrix4d::Identity());
}


Eigen::Vector3d Frame::projectIntoCamera(const Eigen::Vector3d& globalCoord){
    Eigen::Matrix4d pose_inverse = m_global_pose.inverse();
    const auto rotation_inv = pose_inverse.block(0,0,3,3);
    const auto translation_inv = pose_inverse.block(0,3,3,1);
    return rotation_inv * globalCoord + translation_inv;
}

bool Frame::contains(const Eigen::Vector2i& img_coord){
    return img_coord[0] < m_width && img_coord[1] < m_height && img_coord[0] >= 0 && img_coord[1] >= 0;
}

Eigen::Vector2i Frame::projectOntoPlane(const Eigen::Vector3d& cameraCoord){
    Eigen::Vector3d projected = (m_intrinsic_matrix*cameraCoord);
    if(projected[2] == 0){
        return Eigen::Vector2i(MINF, MINF);
    }
    projected /= projected[2];
    return (Eigen::Vector2i ((int) round(projected.x()), (int)round(projected.y())));
}

void Frame::addValidPoints(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals)
{

    // We filter out measurements where either point or normal is invalid.
    const unsigned nPoints = points.size();
    m_points.reserve(nPoints);
    m_normals.reserve(nPoints);

    for (size_t i = 0; i < nPoints; i++) {
        const auto& point = points[i];
        const auto& normal = normals[i];

        if ( (point.allFinite() && normal.allFinite()) ) {
            m_points.push_back(point);
            m_normals.push_back(normal);
        }
        else{
            //m_depth_map[i] = MINF;
            m_points.emplace_back(Eigen::Vector3d(MINF, MINF, MINF));
            m_normals.emplace_back(Eigen::Vector3d(MINF, MINF, MINF));
        }
    }
}

bool Frame::WriteMesh(const std::string& filename, std::string color) {
    // Write off file.
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // Write header.
    outFile << "COFF" << std::endl;
    outFile << m_points_global.size() << " " << "0" << " 0" << std::endl;

    // Save vertices.
    for (unsigned int i = 0; i < m_points_global.size(); i++) {
        const auto& vertex = m_points_global[i];
        if (vertex.allFinite())
            outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
                    << color << std::endl;
        else
            outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
    }
    // Close file.
    outFile.close();

    return true;
}

std::vector<Eigen::Vector3d> Frame::computeCameraCoordinates(unsigned int width, unsigned int height){
    double fovX = m_intrinsic_matrix(0, 0);
    double fovY = m_intrinsic_matrix(1, 1);
    double cX = m_intrinsic_matrix(0, 2);
    double cY = m_intrinsic_matrix(1, 2);

    // auto intrinsicInverse = m_intrinsic_matrix.inverse();

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
                pointsTmp[idx] = Eigen::Vector3d((x - cX) / fovX * depth, (y - cY) / fovY * depth, depth);
            }
        }
    }
    return pointsTmp;
}


std::vector<Eigen::Vector3d> Frame::computeNormals(std::vector<Eigen::Vector3d> camera_points, unsigned int width, unsigned int height,  double maxDistance){

    // We need to compute derivatives and then the normalized normal vector (for valid pixels).
    std::vector<Eigen::Vector3d> normalsTmp(width * height);

    for (size_t v = 1; v < height - 1; ++v) {
        for (size_t u = 1; u < width - 1; ++u) {
            unsigned int idx = v*width + u; // linearized index

            const Eigen::Vector3d du = camera_points[idx + 1] - camera_points[idx - 1];
            const Eigen::Vector3d dv = camera_points[idx + width] - camera_points[idx - width];

            if (!du.allFinite() || !dv.allFinite()
                    || du.norm() > maxDistance
                    || dv.norm() > maxDistance) {
                normalsTmp[idx] = Eigen::Vector3d(MINF, MINF, MINF);
                continue;
            }

            normalsTmp[idx] = du.cross(dv);
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

Eigen::Vector2i Frame::findClosestPoint( const unsigned int u, const unsigned int v, Eigen::Vector3d target, const unsigned int range ){
    double lowest_err = 10000;
    int best_u = -1;
    int best_v = -1;

    for (size_t x = std::max((unsigned int)1, u - range); x < std::min(u + range, m_width-1); x++){
        for (size_t y = std::max((unsigned int)1, v - range); y < std::min(v + range, m_height-1); y++){
            double err = (m_normals_global[y*m_width + x].transpose() *
                (m_points_global[ y*m_width + x ] - target)) ;
            if (err < lowest_err){
                best_u = x;
                best_v = y;
                lowest_err = err;
            }
        }
    }
    
    return Eigen::Vector2i(best_u, best_v);
}

Eigen::Vector2i Frame::findClosestDistancePoint( const unsigned int u, const unsigned int v, Eigen::Vector3d target, const unsigned int range ){
    double lowest_err = 10000;
    int best_u = -1;
    int best_v = -1;

    for (unsigned int x = std::max((unsigned int)0, u - range); x < std::min(u + range, m_width); x++){
        for (unsigned int y = std::max((unsigned int)0, v - range); y < std::min(v + range, m_height); y++){
            double err = (m_points_global[ y*m_width + x ] - target).norm() ;
            if (err < lowest_err){
                best_u = x;
                best_v = y;
                lowest_err = err;
            }
        }
    }
    return Eigen::Vector2i(best_u, best_v);
}

void Frame::applyGlobalPose(Eigen::Matrix4d& estimated_pose){
    Eigen::Matrix3d rotation = estimated_pose.block(0,0,3,3);

    m_points_global  = transformPoints(m_points, estimated_pose);
    m_normals_global = rotatePoints(m_normals, rotation);
}

std::vector<Eigen::Vector3d> Frame::transformPoints(std::vector<Eigen::Vector3d>& points, Eigen::Matrix4d& transformation){
    const Eigen::Matrix3d rotation = transformation.block(0,0,3,3);
    const Eigen::Vector3d translation = transformation.block(0,3,3,1);
    std::vector<Eigen::Vector3d> transformed (points.size());

    for( size_t idx = 0; idx < points.size(); ++idx){
        if(points[idx].allFinite())
            transformed[idx] = rotation * points[idx] + translation;
        else
            transformed[idx] = (Eigen::Vector3d(MINF, MINF, MINF));
    }
    return transformed;
}

std::vector<Eigen::Vector3d> Frame::rotatePoints(std::vector<Eigen::Vector3d>& points, Eigen::Matrix3d& rotation){
    std::vector<Eigen::Vector3d> transformed (points.size());

    for( size_t idx = 0; idx < points.size(); ++idx){
        if(points[idx].allFinite())
            transformed[idx] = rotation * points[idx];
        else
            transformed[idx] = (Eigen::Vector3d(MINF, MINF, MINF));
    }
    return transformed;
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

const Eigen::Matrix4d& Frame::getGlobalPose() const{
    return m_global_pose;
}

void Frame::setGlobalPose(const Eigen::Matrix4d& pose) {
    m_global_pose = pose;
    applyGlobalPose(m_global_pose);
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

double *Frame::getRawDepthMap() const {
    return _rawDepthMap;
}
