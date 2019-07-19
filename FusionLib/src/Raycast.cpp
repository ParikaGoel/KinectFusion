#include <MeshWriter.h>
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

    std::vector<Eigen::Vector3d> vertices (width*height);
    std::vector<double> depthMap (width*height);

    const Eigen::Vector3d volumeRange(volumeSize.x()*voxelScale,volumeSize.y()*voxelScale,volumeSize.z()*voxelScale);

    for( size_t y =0;y<height;y++){
        for(size_t x=0;x< width;x++) {

            //calculate Normalized Direction
            auto direction = calculateRayDirection(x, y, rotationMatrix, currentFrame->getIntrinsics());

            //calculate rayLength
            double rayLength(0);
            //if (!calculateRayLength(rayLength, volumeRange, translation, direction))continue;

            Ray ray (translation, direction);
            if ( ! (volume->intersects( ray, rayLength))) continue;

            Eigen::Vector3d currentPoint;
            if(! calculatePointOnRay(currentPoint, volume, translation,
                                     direction,rayLength))
                continue;

            double TSDF = volume->getTSDF(currentPoint);

            const double maxSearchLength = rayLength + volumeRange.norm();
            //why truncationDistance*0.5?

            Eigen::Vector3d previousPoint;

            for (; rayLength < maxSearchLength; rayLength += truncationDistance * 0.5f) {

                Eigen::Vector3d previousPoint = currentPoint;

                if (!calculatePointOnRay(currentPoint, volume, translation,
                                         direction,rayLength+truncationDistance * 0.5f))
                    continue;
                const double previousTSDF = TSDF;
                TSDF = volume->getTSDF(currentPoint);

                //This equals -ve to +ve in the paper / we cant go from a negative to positive tsdf value as negative is behind the surface
                if (previousTSDF < 0. && TSDF > 0.)break;
                //this equals +ve to -ve in the paper / this means we just crossed a zero value
                if (previousTSDF > 0. && TSDF < 0.) {
                    vertices[x+ y*width] = getZeroCrossing(previousPoint, currentPoint, previousTSDF, TSDF);
                    // Eigen::Vector3d normal_point  = volume->getTSDFGrad(surface_point);
                    depthMap[x + y*width] = vertices[x+ y*width].z();
                }
                //We reached the zero crossing

                //TODO: Calculate vertex : Call getVertexatZeroCrossing with appropriate parameter values

                //TODO: Calculated normal using interpolation method

                //TODO: Calculate color using interpolation method

                //TODO: set global vertex, normal and color into currentFrame
            }
        }
    }

    MeshWriter::toFile("vertices_surface", "255 0 0 255", vertices);
    return true;
}

Eigen::Vector3d Raycast::getZeroCrossing(
        Eigen::Vector3d& prevPoint, Eigen::Vector3d& currPoint,
        double prevTSDF, double currTSDF
){
    return (prevPoint * (-currTSDF) + currPoint * prevTSDF) / (prevTSDF - currTSDF);
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
                                  std::shared_ptr<Volume>& volume,
                                  const Eigen::Vector3d& origin,
                                  const Eigen::Vector3d& direction,
                                  double raylength
) {
    currentPoint = (origin + (direction * raylength));
    return volume->contains(currentPoint);
}

Eigen::Vector3d Raycast::getVertexatZeroCrossing(Eigen::Vector3d& ray_origin,
                                                 Eigen::Vector3d& ray_direction,
                                                 double ray_length){
    return Eigen::Vector3d(ray_origin + ray_direction * ray_length);
}
