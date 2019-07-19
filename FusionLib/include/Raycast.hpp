#pragma once
#include "Volume.hpp"
#include <Frame.h>
#include <memory>

class Raycast {
public:

    //THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters
    bool surfacePrediction(std::shared_ptr<Frame>& currentFrame,std::shared_ptr<Volume>& volume,float truncationDistance);

private:
    /*!
     *
     * @param x
     * @param y
     * @param rotation
     * @param intrinsics
     * @return the normalized direction of the ray which equals the rotation*cameraSpaceCoordinates
     */
    Eigen::Vector3d calculateRayDirection(size_t x, size_t y, const Eigen::Matrix3d& rotation,
                                                const Eigen::Matrix3d& intrinsics);

    /*!
     * This method should not only calculate the currentPoint on the ray but also check wether it is inside the volume 1<=x<=volume.Size.x()-1,...
     * in reference implementation this equals the calculation of the variable grid
     *
     * @param currentPoint pass-by-reference of the point to be calculated
     * @param rayParameter e.g. rayLength
     * @param volumeSize  here volumeSize can be used but currentPoint must be divided by voxelScale
     * @param origin  translation
     * @param direction rotation*currentPoint
     * @return is point in Volume
     */

    bool calculatePointOnRay(Eigen::Vector3d& currentPoint,
                             std::shared_ptr<Volume>& volume,
                             const Eigen::Vector3d& origin,
                             const Eigen::Vector3d& direction,
                             float raylength
    );

    Eigen::Vector3d getVertexAtZeroCrossing(
            const Eigen::Vector3d& prevPoint, const Eigen::Vector3d& currPoint,
            double prevTSDF, double currTSDF);

    Eigen::Vector3i getOriginForInterpolation(const Eigen::Vector3d& point);

    double getTSDFInterpolation(const Eigen::Vector3d& point,
            const std::shared_ptr<Volume>& volume);


    Eigen::Vector3d calculateNormal(const Eigen::Vector3d& gridVertex,
            const std::shared_ptr<Volume>& volume);
};


