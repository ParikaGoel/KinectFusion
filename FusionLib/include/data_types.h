#pragma once

#include "Eigen.h"

typedef unsigned char BYTE;

struct Voxel{
    double tsdf;
    double weight;
    Vector4uc color;
    Voxel():tsdf(0.0f),weight(0.0f),color(0,0,0,0){}
};
