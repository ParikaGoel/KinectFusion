//
// Created by pbo on 08.07.19.
//

#include "Volume.hpp"

const std::vector<Eigen::Vector3d> &Volume::getPoints() const {
    return _points;
}

const Eigen::Vector3i &Volume::getVolumeSize() const {
    return _volumeSize;
}

float Volume::getVoxelScale() const {
    return _voxelScale;
}
