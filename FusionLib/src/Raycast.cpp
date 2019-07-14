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

double get_max_time(const Eigen::Vector3d volumeRange, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction)
{
    double txmax = ((direction.x() > 0 ? volumeRange.x() : 0.f) - origin.x()) / direction.x();
    double tymax = ((direction.y() > 0 ? volumeRange.y(): 0.f) - origin.y()) / direction.y();
    double tzmax = ((direction.z() > 0 ? volumeRange.z() : 0.f) - origin.z()) / direction.z();

    return min(min(txmax, tymax), tzmax);
}

bool Raycast::calculateRayLength(double &rayLength, const Eigen::Vector3d volumeRange, const Eigen::Vector3d &origin,
                                 const Eigen::Vector3d &direction) {
    // TODO implement, check Method signature
    float txmin = ((direction.x() > 0 ? 0.f : volumeRange.x()) - origin.x()) / direction.x();
    float tymin = ((direction.y() > 0 ? 0.f : volumeRange.y()) - origin.y()) / direction.y();
    float tzmin = ((direction.z() > 0 ? 0.f : volumeRange.z()) - origin.z()) / direction.z();

    rayLength =  max(max(txmin, tymin), tzmin);
    if(rayLength >= get_max_time(volumeRange, origin, direction))
    {
        return false;
    }
    return  true;

}

const Eigen::Vector3d
Raycast::calculateRayDirection(int x, int y, Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation,
                               Eigen::Matrix3d intrinsics) {
    // TODO implement, check Method signature
    double fovX = intrinsics(0, 0);
    double fovY = intrinsics(1, 1);
    double cX = m_intrinsics(0, 2);
    double cY = m_intrinsics(1, 2);

    Eigen::Vector3d cameraPoint = Eigen::Vector3d((x - cX) / fovX , (y - cY) / fovY ,1.d);
    Eigen::Vector3d rayDirection = rotation * cameraPoint;
    rayDirection.normalize();

    return rayDirection;
}

bool Raycast::calculateCurrentPointOnRay(Eigen::Vector3d &currentPoint, double &rayParameter,
                                         const Eigen::Vector3d volumeSize, const Eigen::Vector3d &origin,
                                         const Eigen::Vector3d &direction) {
    // TODO implement, check Method signature

    return false;
}

double Raycast::getTSDF(std::shared_ptr<Volume> volume, Eigen::Vector3d position) {
    // TODO implement, check Method signature
    std::pair<double,double> fusionPoints= volume->getPoints()[position.x() + position.y()*volumeSize.x()
                                                               + position.z()*volumeSize.x()*volumeSize.y()];
    const double tsdf = fusionPoints.first;
    return tsdf;
}
