
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <zconf.h>

#include "DepthSensor.h"
#include "icp.h"
#include "Frame.h"

bool writeToFile(std::string filename, int width, int height, std::vector<double> vector){
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << width << "," << height << std::endl;
    for(auto vec : vector)
        outFile << vec<<",";
    return true;
}

int main(){

    // create icp

    // 0. create 1st frame

    // 1. get depth map

    // 2. create Frame (without extrinsics)

    // 3. feed previous frame and created frame to icp
    // frame.setExtrinsics(Sophus::SE3d)
    // frame.applyGlobalTransform()

    // icp.estimatePose(prev, current)

    // frame.writeToMesh()

    // 5. visual in a mesh.off

    double dist_threshold = 0.001;
    double normal_threshold = 0.001;
    icp icp(dist_threshold,normal_threshold);


    DepthSensor sensor;
    sensor.start();

    const Eigen::Matrix3d depthIntrinsics = sensor.GetIntrinsics();
    const unsigned int depthWidth         = sensor.GetDepthImageWidth();
    const unsigned int depthHeight        = sensor.GetDepthImageHeight();

    sensor.ProcessNextFrame();
    std::vector<double> depthMap = sensor.GetDepth();
    Sophus::SE3d init_gl_pose = Sophus::SE3d();
    std::shared_ptr<Frame> prevFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));
    prevFrame->applyGlobalPose(init_gl_pose);

    while(sensor.ProcessNextFrame()){

        sensor.ProcessNextFrame();
        std::vector<double> depthMap = sensor.GetDepth();
        std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));

        icp.estimatePose(prevFrame,currentFrame, 20);

        prevFrame = std::move(currentFrame);

    }
}