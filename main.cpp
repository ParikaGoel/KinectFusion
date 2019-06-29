
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <zconf.h>

#include "DepthSensor.h"
#include "Frame.h"
#include "icp.h"
#include "VirtualSensor.hpp"
#include <DepthMapConverter.hpp>

bool writeToFile(std::string filename, int width, int height, std::vector<double> vector) {
   std::ofstream outFile(filename);
   if (!outFile.is_open()) return false;

   outFile << width << "," << height << std::endl;
        for (auto vec : vector)
            outFile << vec << ",";
        return true;
}

int main(){
    double dist_threshold = 0.001;
    double normal_threshold = 0.001;

    DepthSensor sensor;
    sensor.start();

    const Eigen::Matrix3d depthIntrinsics = sensor.GetIntrinsics();
    const unsigned int depthWidth         = sensor.GetDepthImageWidth();
    const unsigned int depthHeight        = sensor.GetDepthImageHeight();

    sensor.ProcessNextFrame();
    std::vector<double> depthMap = sensor.GetDepth();


    // Frame(std::vector<double> depthMap, const Eigen::Matrix3d& depthIntrinsics, const unsigned int width, const unsigned int height) {

    // Frame frame (depthMap, depthIntrinsics);



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




    icp m_icp();

    //sensor.start();
    sensor.ProcessNextFrame();

    std::cout << "Intrinsics" << std::endl;
    std::cout << std::endl << sensor.GetIntrinsics();
    int width = sensor.GetDepthImageWidth();
    std::cout << std::endl << "WIDTH: " << width;
    int height = sensor.GetDepthImageHeight();
    std::cout << std::endl << "HEIGHT:" << height;

	return 0;
}