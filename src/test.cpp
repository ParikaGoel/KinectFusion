
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>

#include "example.hpp"
#include "DepthSensor.h"


int main(){

    DepthSensor sensor;

    sensor.start();
    sensor.capture();
    float* map = sensor.getDepth();
    for(int i = 0; i < 640*360; i++ )
        if (map[i] > 0.1)
            std::cout <<  map[i] << ",";
    std::cout << std::endl << sensor.getIntrinsics();

    /*
     //Create a simple OpenGL window for rendering:
     window app(1280, 720, "RealSense Pointcloud Example");
     // Construct an object to manage view state
     glfw_state app_state;
     // register callbacks to allow manipulation of the pointcloud
     register_glfw_callbacks(app, app_state);


    rs2::decimation_filter dec;
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;



    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
    auto profile = p.start(cfg);

    // Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // depth = dec.process(depth);
    // depth = depth2disparity.process(depth);
    // depth = spat.process(depth);
    // depth = temp.process(depth);
    // depth = disparity2depth.process(depth);

    rs2::pointcloud pc;

    rs2::points points = pc.calculate(depth);
    const rs2::vertex* vertices = points.get_vertices();

    std::cout << "#points: " << points.size();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();

    std::cout << "width:" << width << std::endl;
    std::cout << "height:" << height << std::endl;


    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);

    for (int i = 0; i < points.size(); ++i ){
        if (vertices[i].z >= (dist_to_center -0.001) && vertices[i].z <= (dist_to_center +0.001)) std::cout << vertices[i].z << " , ";
    }
    std::cout << std::endl << vertices[int(points.size()/2)].z<< std::endl;



    // Print the distance
    std::cout << "The camera is facing an object " << dist_to_center << " meters away \r" << std::endl;

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics(); // Calibration data

    std::cout << "INTRINSICS w" << intrinsics.width << std::endl;
    std::cout << "INTRINSICS h" <<   intrinsics.height << std::endl;
    std::cout << "INTRINSICS ppx" << intrinsics.ppx << std::endl;
    std::cout << "INTRINSICS ppy" << intrinsics.ppy << std::endl;
    std::cout << "INTRINSICS fx" <<  intrinsics.fx << std::endl;
    std::cout << "INTRINSICS fy" <<  intrinsics.fy << std::endl;



     while(app){
         // Wait for the next set of frames from the camera
         auto frames = p.wait_for_frames();

         auto depth = frames.get_depth_frame();

         // Generate the pointcloud and texture mappings
         points = pc.calculate(depth);
         auto color = frames.get_color_frame();
         pc.map_to(color);
         app_state.tex.upload(color);

         // Draw the pointcloud
         draw_pointcloud(app.width(), app.height(), app_state, points);
         // std::cout << "data: " << distances[0];
     }
    */
}