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

    bool intersects(const Ray &r, double& entry_distance) const;

    std::vector<std::pair<double,double>> &getTSDFData() ;

    const Eigen::Vector3d &getOrigin() const;
	const Eigen::Vector3i &getVolumeSize() const;
    float getVoxelScale() const;

    bool contains(const Eigen::Vector3d point);
    Eigen::Vector3d getGlobalCoordinate( int voxelIdx_x, int voxelIdx_y, int voxelIdx_z );

    double getTSDF(Eigen::Vector3d global);
    Eigen::Vector3d getTSDFGrad(Eigen::Vector3d global);

private:
    //_tsdfData contains tuples of tsdf & Weight
    std::vector<std::pair<double,double>> _tsdfData;
    const Eigen::Vector3i _volumeSize;
    const double _voxelScale;
    const Eigen::Vector3d _volumeRange;

    const Eigen::Vector3d _origin;
    const Eigen::Vector3d _maxPoint;
    Eigen::Vector3d bounds[2];

};


