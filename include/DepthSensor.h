//
// Created by frankzl on 28.06.19.
//

#ifndef KINECTFUSION_DEPTHSENSOR_H
#define KINECTFUSION_DEPTHSENSOR_H

#endif //KINECTFUSION_DEPTHSENSOR_H
#include <vector>
#include <librealsense2/rs.hpp>
#include "Sensor.h"

class DepthSensor: public Sensor{
public:

    DepthSensor();

    void start();

    void stop();

    bool ProcessNextFrame();


    rs2::points getPoints();
    rs2::video_frame getColorFrame();
    rs2::frameset getFrameset();
    rs2::pointcloud getPointcloud();

private:
    rs2::pipeline m_pipe;
    rs2::pipeline_profile m_profile;

    rs2::frameset m_frameset;

    rs2::pointcloud m_pc;
    rs2::points m_points;
    const rs2::vertex* m_vertices;

    rs2::decimation_filter dec;
    //rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    // rs2::spatial_filter spat;
    // rs2::temporal_filter temp;
};
