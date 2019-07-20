//
// Created by frankzl on 03.07.19.
//

#include "KinectSensor.h"

KinectSensor::KinectSensor() {
    m_currentIdx = -1;
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    disparity2depth = rs2::disparity_transform(false);
}

void KinectSensor::start() {
     rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    m_profile = m_pipe.start(cfg);

    auto sensor = m_profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

    auto stream = m_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto stream2 = m_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    auto intrinsics    = stream.get_intrinsics(); // Calibration data
    auto colIntrinsics = stream2.get_intrinsics();

    InitDepthIntrinsics(intrinsics.fx, intrinsics.fy, intrinsics.width, intrinsics.height);
    InitColorIntrinsics(colIntrinsics.fx, colIntrinsics.fy, colIntrinsics.width, colIntrinsics.height);
    // TODO
    // InitDepth2ColorExtrinsics(extrinsics.rotation, extrinsics.translation);

    for (auto i = 0; i < 30; ++i) m_pipe.wait_for_frames();
}
void KinectSensor::stop(){
    m_pipe.stop();
}

bool KinectSensor::processNextFrame() {
    m_frameset = m_pipe.wait_for_frames();

    rs2::depth_frame original_depth = m_frameset.get_depth_frame();
    rs2::video_frame color_frame    = m_frameset.get_color_frame();

    m_points = m_pc.calculate(original_depth);
    m_pc.map_to(color_frame);

    m_vertices = m_points.get_vertices();

    std::vector<double> depthMap (m_points.size(), 0);

    for (size_t i = 0; i < m_points.size(); ++i ){
        depthMap[i] = m_vertices[i].z;
    }
    m_depthMap = depthMap;
    return true;
}

rs2::points KinectSensor::getPoints(){
    return m_points;

}

rs2::video_frame KinectSensor::getColorFrame(){
    return m_frameset.get_color_frame();
}

rs2::frameset KinectSensor::getFrameset(){

    return m_frameset;
}

rs2::pointcloud KinectSensor::getPointcloud(){
    return m_pc;
}
