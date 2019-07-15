#pragma once

#include "Volume.hpp"
#include <Frame.h>
#include <memory>
class Fusion {
public:
    Fusion(double max_truncation_dist, double min_truncation_dist, double tsdf_max_weight);

    //THIS method expects frame to hold all camera paramerters as well as the estimated pose --> TODO: check if those values are set or redefine method parameters

    bool reconstructSurface(const std::shared_ptr<Frame>& currentFrame,const std::shared_ptr<Volume>& volume);

private:

    /*bool calculateGlobal2CameraPoint(Eigen::Vector3d &currentCameraPosition, int x, int y, int z,
									 const Eigen::Matrix<double, 3, 3>& rotation,
									 const Eigen::Vector3d& translation, double voxelScale);

    bool pi(Eigen::Vector2i& unhomogenized,const Eigen::Vector3d& homogenized, const Eigen::Matrix3d& intrinsics,int width, int height);
*/
        /*!
         * The original implementation actually takes a raw depth Value, as we already calculated the camereSpacePoints
         * only the normalization has to be done.
         * TODO: move normalization to frame.cpp ; check if the cameraSpaceTransformation in frame.cpp equals the one used in Paper
         * @param cameraSpacePoint
         * @return the normalized cameraSpaceCoordinates
         */
    /*double calculateLamdas(Eigen::Vector2i& cameraSpacePoint,const Eigen::Matrix3d& intrinsics);
    *//*!
     *
     * @param lambda
     * @param cameraPosition
     * @param rawDepthValue
     * @return the signed-distance-function for the specific depth value lambda is based on
     */

    double calculateSDF(Eigen::Vector3d& globalPoint ,Eigen::Vector3d& translation, double rawDepthValue);

    void updateTSDFValue(const std::shared_ptr<Volume>& volume,
            size_t x, size_t y, size_t z,
            double current_tsdf,
            double current_weight);


    double _max_truncation_dist;
    double _min_truncation_dist;
    double _tsdf_max_weight;

};


