//
// Created by frankzl on 03.07.19.
//

#include "DepthSensor.h"

DepthSensor::DepthSensor() : m_currentIdx(-1) {
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    disparity2depth = rs2::disparity_transform(false);
}

void DepthSensor::start() {
    m_profile = m_pipe.start();
    auto sensor = m_profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

    auto stream = m_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics(); // Calibration data

    m_intrinsics(0,0) = intrinsics.fx;
    m_intrinsics(1,1) = intrinsics.fy;
    m_intrinsics(0,2) = intrinsics.width/2;
    m_intrinsics(1,2) = intrinsics.height/2;
    m_intrinsics(2,2) = 1;
}
void DepthSensor::stop(){
    m_pipe.stop();
}
unsigned int DepthSensor::GetDepthImageWidth(){
    unsigned int width = m_intrinsics(0,2)*2;
    return width;
}

unsigned int DepthSensor::GetDepthImageHeight(){
    unsigned int height = m_intrinsics(1,2)*2;
    return height;
}

bool DepthSensor::ProcessNextFrame() {
    m_frameset = m_pipe.wait_for_frames();
    rs2::depth_frame original_depth = m_frameset.get_depth_frame();

    m_points = m_pc.calculate(original_depth);
    m_vertices = m_points.get_vertices();

    std::vector<double> depthMap (m_points.size(), 0);

    for (int i = 0; i < m_points.size(); ++i ){
        depthMap[i] = m_vertices[i].z;
    }
    m_depthMap = depthMap;
    return true;
}

Eigen::Matrix3d DepthSensor::GetIntrinsics(){
    return m_intrinsics;
}

std::vector<double> DepthSensor::GetDepth(){
    return m_depthMap;
}

rs2::points DepthSensor::getPoints(){
    return m_points;

}

rs2::video_frame DepthSensor::getColorFrame(){
    return m_frameset.get_color_frame();
}

rs2::frameset DepthSensor::getFrameset(){

    return m_frameset;
}

rs2::pointcloud DepthSensor::getPointcloud(){
    return m_pc;
}
