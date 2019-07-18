#pragma once

#include "KinectSensor.h"
#include "stb_image_write.h"

class Recorder{
public:

    Recorder(){
        sensor.start();
    }

    void record(unsigned num_frames){

        rs2::colorizer color_map(3);

        std::stringstream color_info;
        std::stringstream depth_info;

        for (size_t i = 0; i < num_frames; i++){
            sensor.processNextFrame();
            auto frame = (color_map.process(sensor.getFrameset().get_depth_frame())).as<rs2::video_frame>();
            auto color_frame = sensor.getFrameset().get_color_frame();

            std::stringstream png_file;
            png_file << PROJECT_DATA_DIR << "/rs-depth/" << i << ".png";
            stbi_write_png(png_file.str().c_str(), frame.get_width(), frame.get_height(),
                           frame.get_bytes_per_pixel(), frame.get_data(), frame.get_stride_in_bytes());

            depth_info << i << " " << "rs-depth/" << i << ".png" << std::endl;

            std::stringstream png_file2;
            png_file2 << PROJECT_DATA_DIR << "/rs-rgb/" << i << ".png";
            stbi_write_png(png_file2.str().c_str(), color_frame.get_width(), color_frame.get_height(),
                           color_frame.get_bytes_per_pixel(), color_frame.get_data(), color_frame.get_stride_in_bytes());

            color_info << i << " " << "rs-color/" << i << ".png" << std::endl;

            std::cout << "Saved " << i << std::endl;
        }

        std::stringstream dir;
        dir << PROJECT_DATA_DIR;

        std::string intro = "# images\n# file\n # timestamp filename\n";
        std::ofstream out (dir.str()+ "/depth.txt");
        out << intro <<depth_info.str();
        std::ofstream out2 (dir.str()+ "/rgb.txt");
        out2 << intro << color_info.str();

        sensor.writeIntrinsicsToFile(dir.str());
        sensor.stop();
    }

private:
    KinectSensor sensor;
};