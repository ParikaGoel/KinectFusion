
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>
#include "SimpleMesh.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <zconf.h>
#include <Volume.hpp>
#include <Fusion.hpp>
#include <Raycast.hpp>

#include "VirtualSensor.h"
#include "icp.h"
#include "Frame.h"

//TODO this should be moved to one File containing all data_declarations class
struct Config{

public:
    Config(const double dist_threshold, const double normal_threshold, const size_t neighbor_range, const double truncationDistance,const double voxelScale, const int x,const int y, const int z):
    m_dist_threshold(dist_threshold),
    m_normal_threshold(normal_threshold),
    m_neighbor_range(neighbor_range),
    m_truncationDistance(truncationDistance),
    m_voxelScale(voxelScale),
    m_volumeSize(x,y,z)
    {};

    const double m_dist_threshold;
    const double m_normal_threshold;
    const size_t m_neighbor_range;
    const double m_truncationDistance;
    const double m_voxelScale;
    Eigen::Vector3i m_volumeSize;

};

bool writeToFile(std::string filename, int width, int height, std::vector<double> vector){
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << width << "," << height << std::endl;
    for(auto vec : vector)
        outFile << vec<<",";
    return true;
}

bool process_frame( std::shared_ptr<Frame> prevFrame,std::shared_ptr<Frame> currentFrame, std::shared_ptr<Volume> volume,const Config& config)
{
    // STEP 1: estimate Pose
    icp icp(config.m_dist_threshold,config.m_normal_threshold, config.m_neighbor_range);

    //TODO give some bool return if icp.estimate was succesfull
   /* if(!icp.estimatePose(prevFrame,currentFrame, 20)){
        throw "ICP Pose Estimation failed";
    };*/

    icp.estimatePose(prevFrame,currentFrame, 20);

    // STEP 2: Surface reconstruction
    //TODO does icp.estimate set the new Pose to currentFrame? Otherwise it needs to be added as function parameter
    Fusion fusion;
    if(!fusion.reconstructSurface(currentFrame,volume, config.m_truncationDistance)){
        throw "Surface reconstruction failed";
    };

    // Step 4: Surface prediction
    Raycast raycast;
    if(!raycast.surfacePrediction(currentFrame,volume, config.m_truncationDistance)){
        throw "Raycasting failed";
    };

    return true;
}

int main(){
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
    //TODO truncationDistance is completly random Value right now
    Config config (0.1,0.5,2,0.5,512,512,512,1.0);

    //Setup Volume
    auto volume = std::make_shared<Volume>(config.m_volumeSize,config.m_voxelScale) ;

    const Eigen::Matrix3d depthIntrinsics = sensor.getDepthIntrinsics();
    const unsigned int depthWidth         = sensor.getDepthImageWidth();
    const unsigned int depthHeight        = sensor.getDepthImageHeight();

    double * depthMap = sensor.getDepth();
    std::shared_ptr<Frame> prevFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));

    Eigen::Matrix4d init_gl_pose = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d current_camera_to_world = init_gl_pose;

    auto normals   = prevFrame->getNormals();
    auto g_normals = prevFrame->getGlobalNormals();
    for (size_t i = 0; i < normals.size(); i++){
        if((normals[i] - g_normals[i]).norm() > 0.001)
        {
            std::cout << "err" << normals[i] << std::endl;
        }
    }

    int i = 0;
    const int iMax = 20;

    std::stringstream ss;
    ss << filenameBaseOut << i << ".off";

    while(sensor.processNextFrame() && i <= iMax){
        
        double* depthMap = sensor.getDepth();
        std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(Frame(depthMap,depthIntrinsics, depthWidth, depthHeight));
        process_frame(prevFrame,currentFrame,volume,config);
        i++;

        ss << filenameBaseOut << i << ".off";
        currentFrame->WriteMesh(ss.str(), "255 0 0 255");
        prevFrame = std::move(currentFrame);
    }
}