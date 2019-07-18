#include "Raycast.hpp"

bool Raycast::surfacePrediction(std::shared_ptr<Frame>& currentFrame,std::shared_ptr<Volume>& volume,float truncationDistance){

    //TODO: implement raycasting

    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose();
    auto rotationMatrix = pose.block(0,0,3,3);
    auto translation = pose.block(0,3,3,1);
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();

    const Eigen::Vector3d volumeRange(volumeSize.x()*voxelScale,volumeSize.y()*voxelScale,volumeSize.z()*voxelScale);

        for( size_t y =0;y<height;y++){
            for(size_t x=0;x< width;x++) {

                //calculate Normalized Direction
                auto direction = calculateRayDirection(x, y, rotationMatrix, currentFrame->getIntrinsics());

                //calculate rayLength
                double rayLength(0);
                if (!calculateRayLength(rayLength, volumeRange, translation, direction))continue;

                Eigen::Vector3d currentPoint;
                calculatePointOnRay(currentPoint, volumeSize, translation,
                                    direction,rayLength,voxelScale);

                double TSDF = getTSDF(volume, currentPoint);

                const double maxSearchLength = rayLength + volumeRange.x() * sqrt(2.0f);
                //why truncationDistance*0.5?

                for (; rayLength < maxSearchLength; rayLength += truncationDistance * 0.5f) {

                    if (!calculatePointOnRay(currentPoint, volumeSize, translation,
                                                    direction,rayLength+truncationDistance * 0.5f,voxelScale))
                        continue;
                    const double previousTSDF = TSDF;
                    TSDF = getTSDF(volume, currentPoint);

                    //This equals -ve to +ve in the paper / we cant go from a negative to positive tsdf value as negative is behind the surface
                    if (previousTSDF < 0. && TSDF > 0.)break;
                    //this equals +ve to -ve in the paper / this means we just crossed a zero value
                    if (previousTSDF > 0. && TSDF < 0.) {
                        // ToDo
                    }
                    //We reached the zero crossing

                    //TODO: Calculate vertex : Call getVertexatZeroCrossing with appropriate parameter values

                    //TODO: Calculated normal using interpolation method

                    //TODO: set global vertex & normal into currentFrame



                }
            }
        }
    return true;
}

double Raycast::interpolateNormals(const Eigen::Vector3d& normal, const std::shared_ptr<Volume>& volume) {
     // TODO implement, check Method signature
    return 0;
}

double get_max_time(const Eigen::Vector3d& volumeRange, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction)
{
    double txMax = ((direction.x() > 0 ? volumeRange.x() : 0.0f) - origin.x()) / direction.x();
    double tyMax = ((direction.y() > 0 ? volumeRange.y(): 0.0f) - origin.y()) / direction.y();
    double tzMax = ((direction.z() > 0 ? volumeRange.z() : 0.0f) - origin.z()) / direction.z();

    return std::min(std::min(txMax, tyMax), tzMax);
}

bool Raycast::calculateRayLength(double &rayLength, const Eigen::Vector3d& volumeRange, const Eigen::Vector3d &origin,
                                 const Eigen::Vector3d &direction) {
    float txMin = ((direction.x() > 0 ? 0.f : volumeRange.x()) - origin.x()) / direction.x();
    float tyMin = ((direction.y() > 0 ? 0.f : volumeRange.y()) - origin.y()) / direction.y();
    float tzMin = ((direction.z() > 0 ? 0.f : volumeRange.z()) - origin.z()) / direction.z();

    rayLength =  std::max(std::max(txMin, tyMin), tzMin);
    double maxTime = get_max_time(volumeRange, origin, direction);
    if(rayLength >= maxTime)
    {
        return false;
    }
    return  true;

}

const Eigen::Vector3d
Raycast::calculateRayDirection(int x, int y, const Eigen::Matrix<double, 3, 3, Eigen::DontAlign>& rotation,
                               const Eigen::Matrix3d& intrinsics) {
    double fovX = intrinsics(0, 0);
    double fovY = intrinsics(1, 1);
    double cX = intrinsics(0, 2);
    double cY = intrinsics(1, 2);

    Eigen::Vector3d cameraPoint = Eigen::Vector3d((x - cX) / fovX , (y - cY) / fovY ,1.0);
    Eigen::Vector3d rayDirection = rotation * cameraPoint;
    rayDirection.normalize();

    return rayDirection;
}

bool Raycast::calculatePointOnRay(Eigen::Vector3d& currentPoint,
                                         const Eigen::Vector3i& volumeSize,
                                         const Eigen::Vector3d& origin,
                                         const Eigen::Vector3d& direction,
                                         double rayParameter,
                                         const double voxelScale) {
    currentPoint = (origin + (direction * rayParameter)) / voxelScale;

    if (currentPoint.x() < 1 || currentPoint.x() >= volumeSize.x() - 1 || currentPoint.y() < 1 ||
            currentPoint.y() >= volumeSize.y() - 1 ||
            currentPoint.z() < 1 || currentPoint.z() >= volumeSize.z() - 1)
        return  false;

    return true;
}

double Raycast::getTSDF(std::shared_ptr<Volume>& volume, Eigen::Vector3d position) {
    Eigen::Vector3i currentPosition;
    currentPosition.x() = int(position.x());
    currentPosition.y() = int(position.y());
    currentPosition.z() = int(position.z());

    Eigen::Vector3i volumeSize = volume->getVolumeSize();
    std::pair<double,double> fusionPoints= volume->getTSDFData()[currentPosition.x() + currentPosition.y()*volumeSize.x()
                                                               + currentPosition.z()*volumeSize.x()*volumeSize.y()];
    double tsdf = fusionPoints.first;
    return tsdf;
}

Eigen::Vector3d Raycast::getVertexatZeroCrossing(Eigen::Vector3d& ray_origin,
        Eigen::Vector3d& ray_direction,
        double ray_length){
    return Eigen::Vector3d(ray_origin + ray_direction * ray_length);
}
