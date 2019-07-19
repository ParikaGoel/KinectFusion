//
// Created by pbo on 08.07.19.
//

#pragma once
#include "Volume.hpp"
#include <Frame.h>
#include <memory>

class Raycast {
public:

    //THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters
    bool surfacePrediction(std::shared_ptr<Frame> currentFrame,std::shared_ptr<Volume> volume,float truncationDistance);

private:

    double interpolateNormals(const Eigen::Vector3d ,std::shared_ptr<Volume> volume );

    /*!
     *
     * @param rayLength  pass-by reference, the variable that should later hold the rayLength
     * @param volumeRange  the volumeSize scaled by the voxelscale, this "equals" the biggest possible distance of the ray
     * @param origin    the origin off the camera equals the translation
     * @param direction  the direction equals the rotation * pixel position
     * @return
     */
    bool calcuateEntryParameter(double &rayLength, const Eigen::Vector3d volumeRange, const Eigen::Vector3d &origin,
                                const Eigen::Vector3d &direction);
    /*!
     *
     * @param x
     * @param y
     * @param rotation
     * @param intrinsics
     * @return the normalized direction of the ray which equals the rotation*cameraSpaceCoordinates
     */
    const Eigen::Vector3d calculateRayDirection(int x,int y,Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation, Eigen::Matrix3d intrinsics);

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

    bool calculateCurrentPointOnRay(Eigen::Vector3d& currentPoint, double& rayParameter, const Eigen::Vector3d volumeSize,const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);

    /*!
     * Attention/TODO: make sure the correct tsdf is retrieved as the mapping is dependant on the for loops in fusion
     *
     * @param volume
     * @param position
     * @return the tsdf from the map in Volume at position, check the way it is done in Fusion
     */
    double getTSDF(std::shared_ptr<Volume> volume, Eigen::Vector3d position);
};


