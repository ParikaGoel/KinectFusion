#include "VirtualSensor.h"

#include "Eigen.h"

#include <librealsense2/rs.hpp>

class DepthSensor {
public:

    DepthSensor() : m_currentIdx(-1) {
    }

    ~DepthSensor() {
        // SAFE_DELETE_ARRAY(m_depthFrame);
    }

    void start(){
        m_profile = m_pipe.start();
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

    bool capture() {
        rs2::frameset frames = m_pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        auto points = pc.calculate(depth);
        vertices = points.get_vertices();
        float depthMap [points.size()];

        for (int i = 0; i < points.size(); ++i ){
            depthMap[i] = vertices[i].z;
        }
        m_depthMap = depthMap;
        return true;
    }

    Eigen::Matrix3d getIntrinsics(){
        return m_intrinsics;
    }

    float* getDepth(){
        return m_depthMap;
    }

private:
    rs2::pipeline m_pipe;
    rs2::pipeline_profile m_profile;

    rs2::pointcloud pc;
    const rs2::vertex* vertices;

    Eigen::Matrix3d m_intrinsics;
    float* m_depthMap;
    int m_currentIdx;
};