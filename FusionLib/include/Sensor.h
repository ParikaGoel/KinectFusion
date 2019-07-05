//
// Created by frankzl on 04.07.19.
//


#ifndef KINECTFUSION_SENSOR_H
#define KINECTFUSION_SENSOR_H


#include <Eigen/Core>
#include <fstream>

class Sensor{
public:
    unsigned int GetDepthImageWidth(){
        return m_depthWidth;
    }

    unsigned int GetDepthImageHeight(){
        return m_depthHeight;
    }

    virtual bool ProcessNextFrame() = 0;

    const Eigen::Matrix3d GetIntrinsics(){
        const Eigen::Matrix3d intr = m_intrinsics;
        return intr;
    }

    std::vector<double> GetDepth(){
        const std::vector<double> depth = m_depthMap;
        return depth;
    }

    bool WriteDepthToFile(std::string directory){
        std::ofstream outFile(directory + "/depth" + std::to_string( m_currentIdx )+ ".txt");
        if (!outFile.is_open()) return false;

        outFile << m_depthWidth << " " << m_depthHeight << std::endl;
        for(int i = 0; i < m_depthMap.size() - 1; ++i) {
            outFile << m_depthMap[i] << " ";
        }
        outFile << m_depthMap[m_depthMap.size() - 1];

        return true;
    }

    bool WriteIntrinsicsToFile(std::string directory){
        std::ofstream outFile(directory + "/intrinsics.txt");
        if (!outFile.is_open()) return false;
        outFile << m_intrinsics(0,0) << " "
            << m_intrinsics(1,1) << " "
            << m_intrinsics(0,2) << " "
            << m_intrinsics(1,2)
            ;
    }

    int GetCurrentFrameCnt(){return m_currentIdx;}

protected:

    void InitIntrinsics(double fx, double fy, int width, int height){
        m_intrinsics(0,0) = fx;
        m_intrinsics(1,1) = fy;
        m_intrinsics(0,2) = width/2;
        m_intrinsics(1,2) = height/2;
        m_intrinsics(2,2) = 1;
    }

    Eigen::Matrix3d m_intrinsics;
    std::vector<double> m_depthMap;
    unsigned int m_depthWidth;
    unsigned int m_depthHeight;
    int m_currentIdx = -1;
};

#endif //KINECTFUSION_SENSOR_H
