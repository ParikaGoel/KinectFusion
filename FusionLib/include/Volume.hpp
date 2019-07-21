#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include "data_types.h"

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

    bool intersects(const Ray &r, float& entry_distance) const;

    std::vector<Voxel>& getVoxelData();

    const Eigen::Vector3d &getOrigin() const;

	const Eigen::Vector3i &getVolumeSize() const;
    float getVoxelScale() const;

    bool contains(const Eigen::Vector3d point);
    Eigen::Vector3d getGlobalCoordinate( int voxelIdx_x, int voxelIdx_y, int voxelIdx_z );

    double getTSDF(Eigen::Vector3d global);
    double getTSDF( int x, int y, int z);

    Vector4uc getColor(Eigen::Vector3d global);

    Eigen::Vector3d getTSDFGrad(Eigen::Vector3d global);

private:
    //_voxelData contains color, tsdf & Weight
    std::vector<Voxel> _voxelData;
    const Eigen::Vector3i _volumeSize;
    const double _voxelScale;
    const Eigen::Vector3d _volumeRange;

    const Eigen::Vector3d _origin;
    const Eigen::Vector3d _maxPoint;
    Eigen::Vector3d bounds[2];

};


