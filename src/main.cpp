
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>

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