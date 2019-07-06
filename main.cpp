
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
    std::string filenameIn = PROJECT_DATA_DIR + std::string("/rgbd_dataset_freiburg1_xyz/");
    std::string filenameBaseOut = PROJECT_DATA_DIR + std::string("/results/mesh_");

    // Load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    // We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
    sensor.processNextFrame();

    // Setup the optimizer.

    double dist_threshold = 0.1;
    double normal_threshold = 1;
    icp icp(dist_threshold,normal_threshold);





    const Eigen::Matrix3d depthIntrinsics = sensor.getDepthIntrinsics();
    const unsigned int depthWidth         = sensor.getDepthImageWidth();
    const unsigned int depthHeight        = sensor.getDepthImageHeight();

//    sensor.ProcessNextFrame();
    double * depthMap = sensor.getDepth();
    Sophus::SE3d init_gl_pose = Sophus::SE3d();
    std::shared_ptr<Frame> prevFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));
    prevFrame->applyGlobalPose(init_gl_pose);

    int i = 0;
    const int iMax = 3;
    while(sensor.processNextFrame() && i <= iMax){

        double* depthMap = sensor.getDepth();
        std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));

        icp.estimatePose(prevFrame,currentFrame, 20);

        prevFrame = std::move(currentFrame);

        i++;

    }


}