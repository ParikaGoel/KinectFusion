#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>

class Volume {
public:
    Volume(const Eigen::Matrix<size_t,3,1>& volumeSize, double voxelScale, const double initial_tsdf);

    ~Volume()= default;

    const std::vector<std::pair<double,double>> &getData() ;

    const std::pair<double,double>& getVoxelData(size_t x, size_t y, size_t z);

    void updateVoxelData(size_t x, size_t y, size_t z, double current_tsdf, double current_weight);

    const Eigen::Matrix<size_t,3,1> &getVolumeSize() const;

    float getVoxelScale() const;

private:
    //tsdf_data contain tuple of tsdf & weight value for each point p in the global model
    // vector size would be x * y * z
    std::vector<std::pair<double,double>> _tsdfData;

    // Overall size of the volume in mm
    Eigen::Matrix<size_t,3,1> _volumeSize;

    // Amount of mm one voxel will represent in each dimension
    double _voxelScale;

};


