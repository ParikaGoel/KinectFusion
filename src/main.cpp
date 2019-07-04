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
    double dist_threshold = 0.001;
    double normal_threshold = 0.001;
    icp m_icp(dist_threshold,normal_threshold);


    DepthSensor sensor;
    sensor.start();

    const Eigen::Matrix3d depthIntrinsics = sensor.GetIntrinsics();
    const unsigned int depthWidth         = sensor.GetDepthImageWidth();
    const unsigned int depthHeight        = sensor.GetDepthImageHeight();

    sensor.ProcessNextFrame();
    std::vector<double> depthMap = sensor.GetDepth();

    /*Frame prevFrame = Frame(depthMap);

    while(sensor.ProcessNextFrame()){

        sensor.ProcessNextFrame();
        std::vector<double> depthMap = sensor.GetDepth();
        Frame prevFrame = Frame();

    }*/







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

    //sensor.start();
    sensor.ProcessNextFrame();

    std::cout << "Intrinsics" << std::endl;
    std::cout << std::endl << sensor.GetIntrinsics();
    int width = sensor.GetDepthImageWidth();
    std::cout << std::endl << "WIDTH: " << width;
    int height = sensor.GetDepthImageHeight();
    std::cout << std::endl << "HEIGHT:" << height;


    std::cout << std::endl << "#points:" << sensor.getPoints().size() << std::endl;

    while (true){
        usleep(5000000);
        std::cout << "capturing" << std::endl;
        sensor.ProcessNextFrame();

        //std::cout << frame.
        auto map = sensor.GetDepth();
        writeToFile("depth.txt", width, height, map);
    }


     // //Create a simple OpenGL window for rendering:
     // window app(1280, 720, "RealSense Pointcloud Example");
     // // Construct an object to manage view state
     // glfw_state app_state;
     // // register callbacks to allow manipulation of the pointcloud
     // register_glfw_callbacks(app, app_state);


    // rs2::decimation_filter dec;
    // dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // rs2::disparity_transform depth2disparity;
    // rs2::disparity_transform disparity2depth(false);
    // // Define spatial filter (edge-preserving)
    // rs2::spatial_filter spat;
    // rs2::temporal_filter temp;




    // rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // // For the color stream, set format to RGBA
    // // To allow blending of the color frame on top of the depth frame
    // cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
    // auto profile = p.start(cfg);

    // Block program until frames arrive
    // rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    // rs2::depth_frame depth = frames.get_depth_frame();

    // depth = dec.process(depth);
    // depth = depth2disparity.process(depth);
    // depth = spat.process(depth);
    // depth = temp.process(depth);
    // depth = disparity2depth.process(depth);

    // rs2::pointcloud pc;

    // Query the distance from the camera to the object in the center of the image
    // float dist_to_center = depth.get_distance(width / 2, height / 2);


    // Print the distance
    // std::cout << "The camera is facing an object " << dist_to_center << " meters away \r" << std::endl;


     // while(app){
     //     // Wait for the next set of frames from the camera
     //     sensor.capture();

     //     // Generate the pointcloud and texture mappings
     //     rs2::points points = sensor.getPoints();

     //     auto color = sensor.getColorFrame();
     //     sensor.getPointcloud().map_to(color);
     //     app_state.tex.upload(color);

     //     // Draw the pointcloud
     //     rs2::colorizer color_map;
     //     app.show(sensor.getFrameset().apply_filter(color_map));
     //     // draw_pointcloud(app.width(), app.height(), app_state, points, 0.0f);
     //     // std::cout << "data: " << distances[0];
     // }

}