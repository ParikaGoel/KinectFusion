//
// Created by pbo on 08.07.19.
//

#pragma once

#include <Eigen/Dense>

class Volume {
public:
    Volume(const Eigen::Vector3i volumeSize, const double voxelScale)
    :m_volumeSize(volumeSize),
    m_voxelScale(voxelScale){};

    ~Volume()= default;


private:
    std::vector<Eigen::Vector3d> m_points;
    Eigen::Vector3i m_volumeSize;
    float m_voxelScale;
};


