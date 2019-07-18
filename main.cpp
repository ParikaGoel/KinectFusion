
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <zconf.h>
#include <Volume.hpp>
#include <Fusion.hpp>
#include <Raycast.hpp>
#include <Recorder.h>
#include <MeshWriter.h>
#include <KinectVirtualSensor.h>

#include "VirtualSensor.h"
#include "icp.h"
#include "Frame.h"


//TODO this should be moved to one File containing all data_declarations class
struct Config{

public:
    Config(const double dist_threshold, const double normal_threshold, const double truncationDistance,
            const Eigen::Vector3d volumeOrigin,const int x,const int y, const int z, const double voxelScale):
    m_dist_threshold(dist_threshold),
    m_normal_threshold(normal_threshold),
    m_truncationDistance(truncationDistance),
    m_voxelScale(voxelScale),
    m_volumeSize(x,y,z),
    m_volumeOrigin(volumeOrigin)
    {};

    const double m_dist_threshold;
    const double m_normal_threshold;
    const double m_truncationDistance;
    const double m_voxelScale;
    Eigen::Vector3i m_volumeSize;
    const Eigen::Vector3d m_volumeOrigin;

};

bool process_frame( size_t frame_cnt, std::shared_ptr<Frame> prevFrame,std::shared_ptr<Frame> currentFrame, std::shared_ptr<Volume> volume,const Config& config)
{
    // STEP 1: estimate Pose
    icp icp(config.m_dist_threshold,config.m_normal_threshold);

    Eigen::Matrix4d estimated_pose = prevFrame->getGlobalPose();
    currentFrame->setGlobalPose(estimated_pose);

    if(!icp.estimatePose(frame_cnt, prevFrame,currentFrame, 3, estimated_pose)){
        throw "ICP Pose Estimation failed";
    };

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

    VirtualSensor sensor;
    std::string filenameIn = PROJECT_DATA_DIR + std::string("/rgbd_dataset_freiburg1_xyz/");

    //std::string filenameIn = PROJECT_DATA_DIR + std::string("/rs9/");

    if (!sensor.init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    //Recorder rec;
    // rec.record(20);

    // 5. visual in a mesh.off
    //std::string filenameIn = PROJECT_DATA_DIR + std::string("/rgbd_dataset_freiburg1_xyz/");
    std::string filenameBaseOut = PROJECT_DATA_DIR + std::string("/results/mesh_");

    // Load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    // VirtualSensor sensor;
    // KinectVirtualSensor sensor(PROJECT_DATA_DIR + std::string("/sample0"), 5 );


    // We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
    sensor.processNextFrame();

    //TODO truncationDistance is completly random Value right now
    const auto volumeOrigin = Eigen::Vector3d (0,0,0);
    Config config (0.1,0.5,0.5, volumeOrigin, 512,512,512,1.0);

    //Setup Volume
    auto volume = std::make_shared<Volume>(config.m_volumeOrigin, config.m_volumeSize,config.m_voxelScale) ;

    Eigen::Matrix3d depthIntrinsics1 = sensor.getDepthIntrinsics();
    // depthIntrinsics1(0,2) = 0;
    // depthIntrinsics1(1,2) = 0;
    const Eigen::Matrix3d depthIntrinsics = depthIntrinsics1;
    const unsigned int depthWidth         = sensor.getDepthImageWidth();
    const unsigned int depthHeight        = sensor.getDepthImageHeight();

    const double* depthMap = &sensor.getDepth()[0];
    BYTE* colors = &sensor.getColorRGBX()[0];

    std::shared_ptr<Frame> prevFrame = std::make_shared<Frame>(Frame(depthMap, depthIntrinsics, depthWidth, depthHeight));

    std::cout << std::endl;
    auto map = prevFrame->getDepthMap();

    std::vector<Eigen::Vector3d> test(depthWidth*depthHeight);
    for (size_t x = 0; x < depthWidth*depthHeight; x++) {
            test[x] = Eigen::Vector3d(x%depthWidth, int (x/depthWidth), map[x]*1000);
    }

    MeshWriter::toFile("test", colors, test);

    MeshWriter::toFile( "my_mesh", colors, prevFrame->getPoints());

    Eigen::Matrix4d init_gl_pose = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d current_camera_to_world = init_gl_pose;

    int i = 0;
    const int iMax = 45;

    std::stringstream ss;
    ss << filenameBaseOut << i << ".off";
    prevFrame->WriteMesh(ss.str(), "0 0 255 100");

    while(sensor.processNextFrame() && i <= iMax){

        const double* depthMap = &sensor.getDepth()[0];
        std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(Frame(depthMap,depthIntrinsics, depthWidth, depthHeight));
        process_frame(i,prevFrame,currentFrame,volume,config);

        std::stringstream ss;
        ss << filenameBaseOut << i << ".off";

        std::cout << "Writing Mesh " << i << std::endl;

        currentFrame->WriteMesh(ss.str(), "255 0 0 255");

        prevFrame = std::move(currentFrame);
        i++;
    }
}
