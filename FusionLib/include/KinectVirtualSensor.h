//
// Created by frankzl on 15.07.19.
//

#pragma once

#include "VirtualSensor.h"

typedef unsigned char BYTE;

class KinectVirtualSensor: public VirtualSensor{

public:
    KinectVirtualSensor(){
        m_currentIdx = -1;
        m_increment = 1;
    }

    bool init(const std::string& datasetDir){
        LoadDepthIntrinsics(datasetDir);
        LoadColorIntrinsics(datasetDir);
        VirtualSensor::init(datasetDir, false);
    }

    bool processNextFrame() {

        if (m_currentIdx == -1) m_currentIdx = 0;
        else m_currentIdx += m_increment;

        if ((unsigned int)m_currentIdx >= (unsigned int)m_filenameColorImages.size()) return false;

        std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_filenameColorImages.size() << "]" << std::endl;

        FreeImageB rgbImage;
        rgbImage.LoadImageFromFile(m_baseDir + m_filenameColorImages[m_currentIdx]);
        memcpy(m_colorFrame, rgbImage.data, 4 * m_colorImageWidth * m_colorImageHeight);
        std::cout<<"loaded image";

        // depth images are scaled by 5000 (see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
        //FreeImageU16F dImage;
        //dImage.LoadImageFromFile(m_baseDir + m_filenameDepthImages[m_currentIdx]);
        LoadDepthFile(m_baseDir + m_filenameDepthImages[m_currentIdx]);
        return true;
    }

    bool LoadDepthIntrinsics(const std::string &datasetDir){
        std::ifstream file(datasetDir+ "depth_intrinsics.txt");
        std::string line;
        std::getline(file, line);

        std::vector<std::string> intrinsics = SplitOnWhitespace(line);

        double fx = atof(intrinsics.at(0).c_str());
        double fy = atof(intrinsics.at(1).c_str());
        int width  = std::stoi(intrinsics.at(2));
        int height = std::stoi(intrinsics.at(3));
        double ppx  = atof(intrinsics.at(4).c_str());
        double ppy  = atof(intrinsics.at(5).c_str());

        InitDepthIntrinsics(fx, fy, ppx, ppy);

        m_depthImageWidth  = width;
        m_depthImageHeight = height;

        return true;
    }

    bool LoadColorIntrinsics(const std::string &datasetDir){
        std::ifstream file(datasetDir + "color_intrinsics.txt");
        std::string line;
        std::getline(file, line);

        std::vector<std::string> intrinsics = SplitOnWhitespace(line);

        double fx = atof(intrinsics.at(0).c_str());
        double fy = atof(intrinsics.at(1).c_str());
        int width  = std::stoi(intrinsics.at(2));
        int height = std::stoi(intrinsics.at(3));
        double ppx  = atof(intrinsics.at(4).c_str());
        double ppy  = atof(intrinsics.at(5).c_str());

        InitColorIntrinsics(fx, fy, ppx, ppy);

        m_colorImageWidth  = width;
        m_colorImageHeight = height;

        return true;
    }

   bool LoadDepthFile(std::string filename) {
       std::ifstream file(filename);
       std::string line;
       std::getline(file, line);

       std::vector<std::string> info = SplitOnWhitespace(line);
       m_depthImageWidth = std::stoi(info.at(0));
       m_depthImageHeight = std::stoi(info.at(1));

       std::getline(file, line);
       std::vector<std::string> points = SplitOnWhitespace(line);

       // std::vector<double> cast_points(points.size());

       for (int i=0; i< points.size(); i++)
       {
           m_depthFrame[i] = atof(points.at(i).c_str());
           //cast_points[i] = num;
       }

       return true;
   }

private:
    std::vector<std::string> SplitOnWhitespace(std::string line){
        std::vector<std::string> result;
        std::istringstream iss(line);
        for(std::string s; iss >> s; )
            result.push_back(s);
        return result;
    }
};
