//
// Created by pbo on 08.07.19.
//

#include "Volume.hpp"

Volume::Volume(const Eigen::Vector3i volumeSize, const double voxelScale)
        : _points(), _volumeSize(volumeSize), _voxelScale(voxelScale) {

    _points.reserve(volumeSize.x() * volumeSize.y() * volumeSize.z());
    for (int z = 0;z<volumeSize.z();z++)
        for( int y =0;y<volumeSize.y();y++)
            for(int x=0;x< volumeSize.x();x++)
                _points.emplace_back(std::pair<double,double>(0,0));

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


