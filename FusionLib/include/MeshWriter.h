#pragma once

#include <string>
#include <fstream>
#include <memory>
#include <iostream>
#include "Frame.h"
#include "Volume.hpp"
#include <Eigen/Core>
#include "data_types.h"


class MeshWriter{
public:

    struct Point{
        Point(double x, double y, double z):_x(x),_y(y),_z(z){}
        double _x,_y,_z;
        std::string print(std::string color){
            std::stringstream s;
            s<<" "<<_x<<" "<<_y<<" "<<_z<<" "<<color<<std::endl;
            return s.str();
        }  };
    struct Square{
        Square(int startIdx,double idx1, double idx2, double idx3, double idx4):_startIdx(startIdx),_idx1(idx1+startIdx),_idx2
                (idx2+startIdx),_idx3(idx3+startIdx),_idx4(idx4+startIdx){

        }

        double _startIdx,_idx1,_idx2,_idx3,_idx4;
        std::string print(std::string color){
            std::stringstream s;
            s<<"4"<<" "<<_idx1<<" "<<_idx2<<" "<<_idx4<<" "<<_idx3<<" "<<color<<std::endl;
            return s.str();
        }
    };
    struct Voxel{
        Voxel(double x, double y, double z, double voxelScale,int startIdx){
            voxelScale /= 2.;
            vertices.push_back({x,y,z});
            vertices.push_back({x+voxelScale,y,z});
            vertices.push_back({x,y,z+voxelScale});
            vertices.push_back({x+voxelScale,y,z+voxelScale});
            vertices.push_back({x,y+voxelScale,z});
            vertices.push_back({x+voxelScale,y+voxelScale,z});
            vertices.push_back({x,y+voxelScale,z+voxelScale});
            vertices.push_back({x+voxelScale,y+voxelScale,z+voxelScale});

            planes.push_back(Square(startIdx,0,1,2,3));
            planes.push_back(Square(startIdx,0,2,4,6));
            planes.push_back(Square(startIdx,0,1,4,5));
            planes.push_back(Square(startIdx,1,3,5,7));
            planes.push_back(Square(startIdx,2,3,6,7));
            planes.push_back(Square(startIdx,4,5,6,7));
        }
        std::string printVertices(std::string color){
            std::stringstream s;
            for(Point p:vertices)
                s<<p.print(color);
            return s.str();
        }
        std::string printPlanes(std::string color){
            std::stringstream s;
            for(Square p:planes)
                s<<" "<< p.print(color);
            return s.str();
        }
        std::vector<Point> vertices;
        std::vector<Square> planes;


    };
    static bool toFile(std::string filename,    Volume& v, int step_size = 1, std::string borderColor = "0 255 0"){

        std::string filenameBaseOut = PROJECT_DIR + std::string("/results/");

        // Write off file.
        std::ofstream outFile(filenameBaseOut + filename + ".off");
        if (!outFile.is_open()){
            return false;
        }
        auto volumeSize = v.getVolumeSize();
        // Save vertices.
        std::vector<Voxel> voxels;
        std::vector<std::string> colors;

        Eigen::Vector3d red(255, 0, 0);
        Eigen::Vector3d blue(0, 0, 255);

        int idx=0;
        auto origin = v.getOrigin();
        for (int z = 0;z<volumeSize.z();z+=step_size) {
            for (int y = 0; y < volumeSize.y(); y+=step_size) {
                for (int x = 0; x < volumeSize.x(); x+=step_size) {
                    auto voxel = v.getVoxelData()[x+y*volumeSize.x()+z*volumeSize.x()*volumeSize.y()];
                    if(voxel.weight == 0. || std::abs(voxel.tsdf) >= 0.9999){
                        continue;
                    }
                    voxels.push_back(Voxel(v.getOrigin().x()+x*v.getVoxelScale(),v.getOrigin().y()+y*v.getVoxelScale
                            (),v.getOrigin().z()+z*v.getVoxelScale(),v.getVoxelScale()*step_size,idx));

                    // max value for TSDF 1, min value -1
                    double tsdf_abs = std::abs(voxel.tsdf);
                    Eigen::Vector3i col = ((1-tsdf_abs) * red + tsdf_abs*blue).cast<int>();
                    std::stringstream s;
                    s << col.x() << " "<< col.y() << " "<< col.z();
                    colors.push_back(s.str());
                    idx+=8;
                }
            }
        }

        std::vector<Eigen::Vector3d> borders;
        for (size_t x = 0; x < volumeSize.x(); ++x){
        borders.push_back(v.getOrigin() + Eigen::Vector3d(x, volumeSize.y(), 0)*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(x, volumeSize.y(), volumeSize.z())*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(x, 0, volumeSize.z())*v.getVoxelScale());
        }
        for (size_t y = 0; y < volumeSize.x(); ++y){
            borders.push_back(v.getOrigin() + Eigen::Vector3d(0, y, 0)*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(0, y, volumeSize.z())*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(volumeSize.x(), y, volumeSize.z())*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(volumeSize.x(), y, 0)*v.getVoxelScale());
        }
        for (size_t z = 0; z < volumeSize.x(); ++z){
            borders.push_back(v.getOrigin() + Eigen::Vector3d(0, 0, z)*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(0, volumeSize.y(), z)*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(volumeSize.x(), volumeSize.y(), z)*v.getVoxelScale());
            borders.push_back(v.getOrigin() + Eigen::Vector3d(volumeSize.x(), 0, z)*v.getVoxelScale());
        }

        // Write header.
        outFile << "COFF" << std::endl;
        outFile << voxels.size() *8<< " " << voxels.size()*6 << " 0" <<std::endl;

        for(size_t i = 0; i < voxels.size(); i++){ //Voxel vo:voxels){
            outFile << voxels[i].printVertices(colors[i]);
        }

        for(Voxel vo:voxels) {
            outFile << vo.printPlanes("255 0 0");
        }

        // Close file.
        outFile.close();
        // Write off file.
        std::ofstream outFile2(filenameBaseOut + filename + "_borders.off");
        // Write header.
        outFile2 << "COFF" << std::endl;
        outFile2 << borders.size() << " 0 0" <<std::endl;

        for(Eigen::Vector3d vec: borders)
            outFile2 << vec.x() << " " << vec.y() << " " << vec.z() << " " << borderColor << std::endl;

        outFile2.close();

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