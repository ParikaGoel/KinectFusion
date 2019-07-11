//
// Created by pbo on 08.07.19.
//

#include "Raycast.hpp"

bool Raycast::surfacePrediction(std::shared_ptr<Frame> currentFrame,std::shared_ptr<Volume> volume,float truncationDistance){

    //TODO: implement raycasting

    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose();
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();

    const Eigen::Vector3d volumeRange(volumeSize.x()*voxelScale,volumeSize.y()*voxelScale,volumeSize.z()*voxelScale);

        for( int y =0;y<height;y++){
            for(int x=0;x< width;x++){

                //calculate Normalized Direction
                auto direction = calculateRayDirection(x,y,pose.rotationMatrix(),currentFrame->getIntrinsics());

                //calculate rayLength
                double rayLength(0);
                if(!calculateRayLength(rayLength,volumeRange,pose.translation(),direction))continue;

                Eigen::Vector3d currentPoint;
                calculateCurrentPointOnRay(currentPoint,rayLength,volumeSize,pose.translation(),direction);

                double TSDF = getTSDF(volume,currentPoint);

                const double maxSearchLength = rayLength + volumeRange.x() * sqrt(2.f);
                //why truncationDistance*0.5?

                for(rayLength;rayLength<maxSearchLength;rayLength+=truncationDistance*0.5f){

                    if(!calculateCurrentPointOnRay(currentPoint,rayLength,volumeSize,pose.translation(),direction))continue;
                    const double previousTSDF = TSDF;
                    getTSDF(volume,currentPoint);

                    //This equals -ve to +ve in the paper / we cant go from a negative to positive tsdf value as negative is behind the surface
                    if(previousTSDF<0. && TSDF>0.)break;
                    //this equals +ve to -ve in the paper / this means we just crossed a zero value
                    if(previousTSDF>0. && TSDF<0.)
                        //We reached the zero crossing

                        //TODO: Calculate vertex

                        //TODO: Calculated normal using interpolation method

                        //TODO: set global vertex & normal into currentFrame



                    }



                }





            }
        }



    return true;
}

double Raycast::interpolateNormals(const Eigen::Vector3d, std::shared_ptr<Volume> volume) {
     // TODO implement, check Method signature
    return 0;
}

bool Raycast::calculateRayLength(double &rayLength, const Eigen::Vector3d volumeRange, const Eigen::Vector3d &origin,
                                 const Eigen::Vector3d &direction) {
    // TODO implement, check Method signature

    return false;
}

const Eigen::Vector3d
Raycast::calculateRayDirection(int x, int y, Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation,
                               Eigen::Matrix3d intrinsics) {
    // TODO implement, check Method signature

    return Eigen::Vector3d();
}

bool Raycast::calculateCurrentPointOnRay(Eigen::Vector3d &currentPoint, double &rayParameter,
                                         const Eigen::Vector3d volumeSize, const Eigen::Vector3d &origin,
                                         const Eigen::Vector3d &direction) {
    // TODO implement, check Method signature

    return false;
}

double Raycast::getTSDF(std::shared_ptr<Volume> volume, Eigen::Vector3d position) {
    // TODO implement, check Method signature

    return 0;
}
