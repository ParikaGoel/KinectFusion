#include "Volume.hpp"

Volume::Volume(const Eigen::Matrix<size_t,3,1>& volumeSize, const double voxelScale, const double initial_tsdf)
        : _tsdfData(), _volumeSize(volumeSize), _voxelScale(voxelScale) {

    _tsdfData.reserve(volumeSize.x() * volumeSize.y() * volumeSize.z());
    for (size_t z = 0;z<volumeSize.z();z++)
        for( size_t y =0;y<volumeSize.y();y++)
            for(size_t x=0;x< volumeSize.x();x++)
                _tsdfData.emplace_back(std::pair<double,double>(initial_tsdf,0));

}

void Volume::updateVoxelData(size_t x, size_t y, size_t z, double updated_tsdf, double updated_weight){
    size_t index = z * (_volumeSize.x() + _volumeSize.y()) + y * _volumeSize.x() + x;

    _tsdfData[index].first = updated_tsdf;
    _tsdfData[index].second = updated_weight;
}

const std::pair<double,double>& Volume::getVoxelData(size_t x, size_t y, size_t z){
    size_t index = z * (_volumeSize.x() + _volumeSize.y()) + y * _volumeSize.x() + x;
    return _tsdfData[index];
}

const std::vector<std::pair<double, double>> &Volume::getData()  {
	return _tsdfData;
}

const Eigen::Matrix<size_t,3,1> &Volume::getVolumeSize() const {
	return _volumeSize;
}

float Volume::getVoxelScale() const {
	return _voxelScale;
}


