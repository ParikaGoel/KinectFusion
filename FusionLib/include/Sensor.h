#pragma once

#include <Eigen/Core>
#include <fstream>

typedef unsigned char BYTE;

class Sensor{
public:
    unsigned int getDepthImageWidth(){
        return m_depthImageWidth;
    }

    unsigned int getDepthImageHeight(){
        return m_depthImageHeight;
    }

    virtual bool processNextFrame() = 0;

    const Eigen::Matrix3d getDepthIntrinsics(){
        const Eigen::Matrix3d intr = m_depthIntrinsics;
        return intr;
    }

    const Eigen::Matrix3d getColorIntrinsics(){
        const Eigen::Matrix3d intr = m_colorIntrinsics;
        return intr;
    }

    std::vector<double> getDepth(){
        const std::vector<double> depth = m_depthMap;
        return depth;
    }

    bool writeDepthToFile(std::string directory){
        std::ofstream outFile(directory + "/depth" + std::to_string( m_currentIdx )+ ".txt");
        if (!outFile.is_open()) return false;

        outFile << m_depthImageWidth << " " << m_depthImageHeight << std::endl;
        for(unsigned int i = 0; i < m_depthMap.size() - 1; ++i) {
            outFile << m_depthMap[i] << " ";
        }
        outFile << m_depthMap[m_depthMap.size() - 1];

        return true;
    }

    bool writeIntrinsicsToFile(std::string directory){
        std::ofstream outFile(directory + "/depth_intrinsics.txt");
        if (!outFile.is_open()) return false;
        outFile << m_depthIntrinsics(0,0) << " "
                << m_depthIntrinsics(1,1) << " "
                << m_depthIntrinsics(0,2) << " "
                << m_depthIntrinsics(1,2)
                ;

        std::ofstream outFile2(directory + "/color_intrinsics.txt");
        if (!outFile2.is_open()) return false;
        outFile2 << m_depthIntrinsics(0,0) << " "
                 << m_depthIntrinsics(1,1) << " "
                 << m_depthIntrinsics(0,2) << " "
                 << m_depthIntrinsics(1,2)
                ;
    }

    int GetCurrentFrameCnt(){return m_currentIdx;}

private:
    void InitIntrinsics(double fx, double fy, double ppx, double ppy, Eigen::Matrix3d& intrinsics){
        intrinsics.setZero();
        intrinsics(0,0) = fx;
        intrinsics(1,1) = fy;
        intrinsics(0,2) = ppx;
        intrinsics(1,2) = ppy;
        intrinsics(2,2) = 1;
    }

protected:

    void InitDepthIntrinsics(double fx, double fy, double ppx, double ppy){
        InitIntrinsics(fx, fy, ppx, ppy, m_depthIntrinsics);
        //m_depthImageWidth  = width;
        //m_depthImageHeight = height;
    }

    void InitColorIntrinsics(double fx, double fy, double width, double height){
        InitIntrinsics(fx, fy, width, height, m_colorIntrinsics);
        m_colorImageWidth = width;
        m_colorImageHeight = height;
    }

    Eigen::Matrix3d m_depthIntrinsics;
    Eigen::Matrix3d m_colorIntrinsics;
    std::vector<double> m_depthMap;
    std::vector<BYTE> m_colorFrame;
    unsigned int m_depthImageWidth;
    unsigned int m_depthImageHeight;
    unsigned int m_colorImageWidth;
    unsigned int m_colorImageHeight;

    int m_currentIdx = -1;
};

typedef unsigned char BYTE;