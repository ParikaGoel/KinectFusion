//
// Created by frankzl on 28.06.19.
//

#ifndef KINECTFUSION_DEPTHSENSOR_H
#define KINECTFUSION_DEPTHSENSOR_H

#endif //KINECTFUSION_DEPTHSENSOR_H
#include "VirtualSensor.h"
#include <vector>

#include <librealsense2/rs.hpp>


class DepthSensor {
public:

    DepthSensor() : m_currentIdx(-1) {
        dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        disparity2depth = rs2::disparity_transform(false);
    }

    ~DepthSensor() {
        // SAFE_DELETE_ARRAY(m_depthFrame);
    }

    void start(){
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

    void stop(){
        m_pipe.stop();
    }

    int GetDepthImageWidth(){
        return m_intrinsics(0,2)*2;
    }

    int GetDepthImageHeight(){
        return m_intrinsics(1,2)*2;
    }

    bool ProcessNextFrame() {
        m_frameset = m_pipe.wait_for_frames();
        rs2::depth_frame original_depth = m_frameset.get_depth_frame();

        // auto depth = dec.process(original_depth);
        // depth = depth2disparity.process(depth);
        // depth = spat.process(depth);
        // depth = temp.process(depth);
        // depth = disparity2depth.process(depth);

        m_points = m_pc.calculate(original_depth);
        m_vertices = m_points.get_vertices();

        std::vector<float> depthMap (m_points.size(), 0);

        for (int i = 0; i < m_points.size(); ++i ){
            depthMap[i] = m_vertices[i].z;
        }
        m_depthMap = depthMap;
        return true;
    }

    Eigen::Matrix3d getIntrinsics(){
        return m_intrinsics;
    }

    std::vector<float> GetDepth(){
        return m_depthMap;
    }

    rs2::points getPoints(){
        return m_points;

    }

    rs2::video_frame getColorFrame(){
        return m_frameset.get_color_frame();
    }

    rs2::frameset getFrameset(){
        return m_frameset;
    }

    rs2::pointcloud getPointcloud(){
        return m_pc;
    }

private:
    rs2::pipeline m_pipe;
    rs2::pipeline_profile m_profile;

    rs2::frameset m_frameset;

    rs2::pointcloud m_pc;
    rs2::points m_points;
    const rs2::vertex* m_vertices;

    Eigen::Matrix3d m_intrinsics;
    std::vector<float> m_depthMap;
    int m_currentIdx;


    rs2::decimation_filter dec;
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;
};
