//
// Created by pbo on 08.07.19.
//

#pragma once

#include "Volume.hpp"
#include <Frame.h>
#include <memory>
class Fusion {
public:
//THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters

    bool reconstructSurface(std::shared_ptr<Frame> currentFrame,std::shared_ptr<Volume> volume,float truncationDistance);

private:

};


