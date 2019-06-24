#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include "Eigen.h"
#include "VirtualSensor.h"



int main()
{

    std::string filenameIn = "data/rgbd_dataset_freiburg1_xyz/";
    std::cout << "Initialize virtual sensor..." << std::endl;

    VirtualSensor sensor;

    if (!sensor.Init(filenameIn) ){
        std::cout << "Failed to initialize sensor!" << std::endl;
        return -1;
    }

    while (sensor.ProcessNextFrame() )
    {
        std::cout << "Next Frame." << std::endl;

    }
    
    return 0;
}

