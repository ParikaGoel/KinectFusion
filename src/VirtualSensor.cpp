//
// Created by frankzl on 04.07.19.
//

#pragma once

#include "VirtualSensor.h"

typedef unsigned char BYTE;

VirtualSensor::VirtualSensor(std::string directory, int step_size) : m_directory(directory), m_step_size(step_size) {
    std::cout << "cnt: " << m_currentIdx;
    LoadIntrinsics();
    std::cout << "cnt: " << m_currentIdx;
    std::string filename = m_directory + "/depth0.txt";
    LoadDepthFile(filename);
    std::cout << "cnt: " << m_currentIdx;
}

bool VirtualSensor::ProcessNextFrame() {

    m_currentIdx += 1;

    std::string filename = m_directory + "/depth" + std::to_string(m_currentIdx) + ".txt";


    std::ifstream file (filename);

    if(file.good()){
        LoadDepthFile(filename);
        return true;
    }

    return false;
}


std::vector<std::string> VirtualSensor::SplitOnWhitespace(std::string line){
    std::vector<std::string> result;
    std::istringstream iss(line);
    for(std::string s; iss >> s; )
        result.push_back(s);
    return result;
}

bool VirtualSensor::LoadDepthFile(std::string filename) {

    std::ifstream file(filename);
    std::string line;
    std::getline(file, line);



    std::vector<std::string> info = SplitOnWhitespace(line);
    m_depthWidth = std::stoi(info.at(0));
    m_depthHeight = std::stoi(info.at(1));

    std::getline(file, line);
    std::vector<std::string> points = SplitOnWhitespace(line);


    std::vector<double> cast_points;

    for (int i=0; i< points.size(); i++)
    {
        double num = atof(points.at(i).c_str());
        cast_points.push_back(num);
    }

    m_depthMap = cast_points;

    return true;
}

bool VirtualSensor::LoadIntrinsics() {
    std::ifstream file(m_directory + "/intrinsics.txt");
    std::cout << m_directory + "/intrinsics.txt";
    std::string line;
    std::cout << "is good?"<< file.good();
    std::getline(file, line);


    std::vector<std::string> intrinsics = SplitOnWhitespace(line);


    double fx = atof(intrinsics.at(0).c_str());
    double fy = atof(intrinsics.at(1).c_str());
    int width  = std::stoi(intrinsics.at(2));
    int height = std::stoi(intrinsics.at(3));

    InitIntrinsics(fx, fy, width, height);
}
