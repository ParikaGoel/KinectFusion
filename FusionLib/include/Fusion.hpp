#pragma once

#include "Volume.hpp"
#include <Frame.h>
#include <memory>
class Fusion {
public:
//THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters

    bool reconstructSurface(const std::shared_ptr<Frame>& currentFrame,const std::shared_ptr<Volume>& volume,double truncationDistance);

private:

    //Reconstruction
    bool calculateGlobal2CameraPoint(Eigen::Vector3d &currentCameraPosition, int x, int y, int z,
									 const Eigen::Matrix3d& rotation,
									 const Eigen::Vector3d& translation, double voxelScale);

    /*!
         * The original implementation actually takes a raw depth Value, as we already calculated the camereSpacePoints
         * only the normalization has to be done.
         * TODO: move normalization to frame.cpp ; check if the cameraSpaceTransformation in frame.cpp equals the one used in Paper
         * @param cameraSpacePoint
         * @return the normalized cameraSpaceCoordinates
         */
    double calculateLamdas(Eigen::Vector2i& cameraSpacePoint,Eigen::Matrix3d intrinsics);
    /*!
     *
     * @param lambda
     * @param cameraPosition
     * @param rawDepthValue
     * @return the signed-distance-function for the specific depth value lambda is based on
     */

    double calculateSDF(double& lambda,Eigen::Vector3d& cameraPosition,double rawDepthValue);

};


