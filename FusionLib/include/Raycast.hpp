#pragma once
#include "Volume.hpp"
#include <Frame.h>
#include <memory>

class Raycast {
public:

    //THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters
    bool surfacePrediction(std::shared_ptr<Frame>& currentFrame,std::shared_ptr<Volume>& volume,float truncationDistance);

private:

    double interpolateNormals(const Eigen::Vector3d& normal, const std::shared_ptr<Volume>& volume);


    /*!
     *
     * @param rayLength  pass-by reference, the variable that should later hold the rayLength
     * @param volumeRange  the volumeSize scaled by the voxelscale, this "equals" the biggest possible distance of the ray
     * @param origin    the origin off the camera equals the translation
     * @param direction  the direction equals the rotation * pixel position
     * @return
     */
    bool calculateRayLength(double &rayLength, const Eigen::Vector3d& volumeRange, const Eigen::Vector3d &origin,
                            const Eigen::Vector3d &direction);
    /*!
     *
     * @param x
     * @param y
     * @param rotation
     * @param intrinsics
     * @return the normalized direction of the ray which equals the rotation*cameraSpaceCoordinates
     */
    const Eigen::Vector3d calculateRayDirection(int x, int y, const Eigen::Matrix<double, 3, 3, Eigen::DontAlign>& rotation,
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
                             double raylength
    );

    /*!
     * @param ray_origin : originating position of vector to the vertex point at zero crossing
     * @param ray_direction : direction of the vector
     * @param ray_length: length of the vector specifying how far the point is from origin
     * @return 3d point of the vertex at zero crossing
     */
    Eigen::Vector3d getVertexatZeroCrossing(Eigen::Vector3d& ray_origin,
                                            Eigen::Vector3d& ray_direction,
                                            double ray_length);

    Eigen::Vector3d getZeroCrossing(
            Eigen::Vector3d& prevPoint, Eigen::Vector3d& currPoint,
            double prevTSDF, double currTSDF
            );
};


