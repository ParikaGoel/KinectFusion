#pragma once

#include <string>
#include <fstream>
#include <memory>
#include <iostream>
#include <Eigen/Core>

#include "Volume.hpp"
#include "Frame.h"
#include "data_types.h"


class MeshWriter{
public:

    static bool toFile(std::string filename,    Volume& v){

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()){
            return false;
        }
        auto volumeSize = v.getVolumeSize();
        // Write header.
        outFile << "COFF" << std::endl;
        outFile << volumeSize.x()*volumeSize.z()*volumeSize.y() << " " << "0" << " 0" <<
                std::endl;
        // Save vertices.
        auto origin = v.getOrigin();
        for (int z = 0;z<volumeSize.z();z++) {

            for (int y = 0; y < volumeSize.y(); y++) {
                for (int x = 0; x < volumeSize.x(); x++) {
                    auto voxel = v.getVoxelData()[x+y*volumeSize.x()+z*volumeSize.x()*volumeSize.y()];
                    if(voxel.weight == 0. || std::abs(voxel.tsdf) >= 0.999){
                        continue;
                    }
                    // if(v.getVoxelData()[x+y*volumeSize.x()+z*volumeSize.x()*volumeSize.y()].tsdf ==0){
                    //     outFile<<x<<" "<<y<<" "<<z<<" "<<"255 255 0 0.5"<<std::endl;
                    // }else{
                    //     outFile<<x<<" "<<y<<" "<<z<<" "<<"0 0 255 0.5"<<std::endl;
                    // }
                    outFile<<x*v.getVoxelScale() + origin.x()<<" "<<y*v.getVoxelScale() + origin.y()<<" "<<z*v.getVoxelScale() + origin.z()<<" "<<"0 0 255 255"<<std::endl;
                }
            }
        }

        // Close file.
        outFile.close();
        return true;
    }

    static bool toFile(std::string filename, BYTE* color,
                       const std::vector<Eigen::Vector3d>& points){

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::cout << filename << std::endl;
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()) return false;

        // Write header.
        outFile << "COFF" << std::endl;
        outFile << points.size() << " " << "0" << " 0" << std::endl;

        // Save vertices.
        for (unsigned int i = 0; i < points.size(); i++) {
            const auto& vertex = points[i];
            if (vertex.allFinite() && vertex.z() > 0)
                outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
                        << (unsigned int)color[i*4] << " " << (unsigned int)color[i*4 +1] <<
                        " " << (unsigned int)color[i*4+2]<< " " << (unsigned int)color[i*4 + 3] <<std::endl;
            else
                outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
        }
        // Close file.
        outFile.close();
        return true;
    }

    static bool toFile(std::string filename, std::string color,
                       const std::vector<Eigen::Vector3d>& points){

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::cout << filename << std::endl;
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()) return false;

        // Write header.
        outFile << "COFF" << std::endl;
        outFile << points.size() << " " << "0" << " 0" << std::endl;

        // Save vertices.
        for (unsigned int i = 0; i < points.size(); i++) {
            const auto& vertex = points[i];
            if (vertex.allFinite())
                outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
                        << color << std::endl;
            else
                outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
        }
        // Close file.
        outFile.close();
        return true;
    }

    static bool toFile(std::string filename,
                       const std::vector<Eigen::Vector3d>& points){

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::cout << filename << std::endl;
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()) return false;

        // Write header.
        outFile << "COFF" << std::endl;
        outFile << points.size() << " " << "0" << " 0" << std::endl;

        int max_col = 256; //*256*256 - 1;
        int step = (int) (max_col / points.size());
        int cur_col = 0;

        // Save vertices.
        for (unsigned int i = 0; i < points.size(); i++) {
            const auto& vertex = points[i];

            int first  = ( cur_col% 256 );
            int second = 256; //int ( cur_col / 256 ) % 256;
            int third  = 256; // int ( cur_col / (256*256) ) % 256;

            cur_col += step;


            if (vertex.allFinite())
                outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
                        << first << " " << second << " " << third << " 255" << std::endl;
            else
                outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
        }
        // Close file.
        outFile.close();
        return true;
    }

    static bool toFile(std::string filename , const std::shared_ptr<Frame>& frame)
    {

        auto global_points = frame->getGlobalPoints();
        auto color_map = frame->getColorMap();

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::cout << filename << std::endl;
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()) return false;

        // Write header.
        outFile << "COFF" << std::endl;
        outFile << global_points.size() << " " << "0" << " 0" << std::endl;

        // Save vertices.
        for (size_t i = 0; i < global_points.size(); i++) {
            const auto& vertex = global_points[i];
            if (vertex.allFinite())
                outFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
                        << int(color_map[i][0]) << " " << int(color_map[i][1]) << " "
                        << int(color_map[i][2]) << " " << int(color_map[i][3]) << std::endl;
            else
                outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
        }
        // Close file.
        outFile.close();

        return true;
    }

};