//
// Created by pbo on 08.07.19.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

class Volume {
public:
    Volume(const Eigen::Vector3i volumeSize, const double voxelScale)
    :_volumeSize(volumeSize),
    _voxelScale(voxelScale){};

    ~Volume()= default;

    const std::vector<Eigen::Vector3d> &getPoints() const;

    const Eigen::Vector3i &getVolumeSize() const;

    float getVoxelScale() const;

private:
    //TODO: _points should not containt points, but tuples of tsdf & Weight
    std::vector<Eigen::Vector3d> _points;
    Eigen::Vector3i _volumeSize;
    float _voxelScale;

};


