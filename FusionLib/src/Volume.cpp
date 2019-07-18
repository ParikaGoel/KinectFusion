//
// Created by pbo on 08.07.19.
//

#include "Volume.hpp"

Ray::Ray(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir) : orig(origin), dir(dir) {
    invdir[0] = 1/dir[0];
    invdir[1] = 1/dir[1];
    invdir[2] = 1/dir[2];
    sign[0] = (invdir.x() < 0);
    sign[1] = (invdir.y() < 0);
    sign[2] = (invdir.z() < 0);
}


Volume::Volume(const Eigen::Vector3d origin, const Eigen::Vector3i volumeSize, const double voxelScale)
        : _maxPoint(voxelScale * volumeSize.cast<double>()),
          _origin(origin), _points(), _volumeSize(volumeSize),
          _voxelScale(voxelScale) {

    _points.reserve(volumeSize.x() * volumeSize.y() * volumeSize.z());
    for (int z = 0;z<volumeSize.z();z++)
        for( int y =0;y<volumeSize.y();y++)
            for(int x=0;x< volumeSize.x();x++)
                _points.emplace_back(std::pair<double,double>(0,0));

}


const Eigen::Vector3d& Volume::getOrigin() const{
    return _origin;
}


std::vector<std::pair<double, double>> &Volume::getPoints()  {
	return _points;
}

const Eigen::Vector3i &Volume::getVolumeSize() const {
	return _volumeSize;
}

float Volume::getVoxelScale() const {
	return _voxelScale;
}


