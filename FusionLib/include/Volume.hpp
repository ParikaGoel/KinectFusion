//
// Created by pbo on 08.07.19.
//

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>

class Ray
{
public:
    Ray(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir);
    Eigen::Vector3d orig, dir, invdir;
    int sign[3];
};

class Volume {
public:
    Volume(const Eigen::Vector3d origin, const Eigen::Vector3i volumeSize, const double voxelScale);
    ~Volume()= default;

    std::vector<std::pair<double,double>> &getPoints() ;

    const Eigen::Vector3d &getOrigin() const;


	const Eigen::Vector3i &getVolumeSize() const;

    float getVoxelScale() const;

private:
    //TODO: _points should not containt points, but tuples of tsdf & Weight
    std::vector<std::pair<double,double>> _points;
    const Eigen::Vector3i _volumeSize;
    const double _voxelScale;

    const Eigen::Vector3d _origin;
    const Eigen::Vector3d _maxPoint;

};


