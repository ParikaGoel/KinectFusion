//
// Created by frankzl on 07.07.19.
//

#include <string>
#include <fstream>
#include <Eigen/Core>

#include "Frame.h"

#ifndef KINECTFUSION_MESHWRITER_H
#define KINECTFUSION_MESHWRITER_H

#endif //KINECTFUSION_MESHWRITER_H

class MeshWriter{
public:


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

    static bool toFile(std::string filename,std::string color, Frame& frame ) {
        return toFile(filename, color, frame.getGlobalPoints());
    }

};
