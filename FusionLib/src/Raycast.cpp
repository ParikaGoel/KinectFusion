#include <MeshWriter.h>
#include "Raycast.hpp"

bool Raycast::surfacePrediction(std::shared_ptr<Frame>& currentFrame,std::shared_ptr<Volume>& volume,float truncationDistance){

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

    for( size_t v =0;v<height;v++){
        for(size_t u=0;u< width;u++) {

            //calculate Normalized Direction
            auto direction = calculateRayDirection(u, v, rotationMatrix, currentFrame->getIntrinsics());

            //calculate rayLength
            float rayLength(0.0f);

            Ray ray (translation, direction);
            if ( ! (volume->intersects( ray, rayLength))) continue;

            Eigen::Vector3d currentPoint;
            if(! calculatePointOnRay(currentPoint, volume, translation,
                                     direction,rayLength))
                continue;

            double currentTSDF = volume->getTSDF(currentPoint);

            const double maxSearchLength = rayLength + volumeRange.norm();

            Eigen::Vector3d previousPoint;

            for (; rayLength < maxSearchLength; rayLength += truncationDistance * 0.5f) {

                Eigen::Vector3d previousPoint = currentPoint;
                const double previousTSDF = currentTSDF;

                if (!calculatePointOnRay(currentPoint, volume, translation,
                                         direction,rayLength+truncationDistance * 0.5f))
                    continue;

                currentTSDF = volume->getTSDF(currentPoint);

                //This equals -ve to +ve in the paper / we cant go from a negative to positive tsdf value as negative is behind the surface
                if (previousTSDF < 0. && currentTSDF > 0.)break;
                //this equals +ve to -ve in the paper / this means we just crossed a zero value
                if (previousTSDF > 0. && currentTSDF < 0.) {
                    /*vertices[u+ v*width] = getVertexAtZeroCrossing(previousPoint, currentPoint, previousTSDF, currentTSDF);
                    // Eigen::Vector3d normal_point  = volume->getTSDFGrad(surface_point);
                    depthMap[u + v*width] = vertices[u+ v*width].z();*/

                    Eigen::Vector3d globalVertex = getVertexAtZeroCrossing(previousPoint, currentPoint, previousTSDF, currentTSDF);

                    Eigen::Vector3d gridVertex = globalVertex / voxelScale;

                    if (gridVertex.x()-1 < 1 || gridVertex.x()+1 >= volumeSize.x() - 1 ||
                            gridVertex.y()-1 < 1 || gridVertex.y()+1 >= volumeSize.y() - 1 ||
                            gridVertex.z()-1 < 1 || gridVertex.z()+1 >= volumeSize.z() - 1)
                        break;

                    Eigen::Vector3d normal = calculateNormal(gridVertex, volume);

                    Vector4uc color;

                    if(std::abs(previousTSDF) < std::abs(currentTSDF)){
                        color = volume->getColor(previousPoint);
                    }
                    else{
                        color = volume->getColor(currentPoint);
                    }

                    currentFrame->setGlobalPoint(globalVertex,u,v);
                    currentFrame->setGlobalNormal(normal,u,v);
                    currentFrame->setColor(color,u,v);

                }
            }
        }
    }

    MeshWriter::toFile("vertices_surface", "255 0 0 255", vertices);
    return true;
}

Eigen::Vector3d Raycast::getVertexAtZeroCrossing(
        const Eigen::Vector3d& prevPoint, const Eigen::Vector3d& currPoint,
        double prevTSDF, double currTSDF)
{
    return (prevPoint * (-currTSDF) + currPoint * prevTSDF) / (prevTSDF - currTSDF);
}

Eigen::Vector3d Raycast::calculateRayDirection(size_t x, size_t y, const Eigen::Matrix3d& rotation,
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
                                  float raylength
) {
    currentPoint = (origin + (direction * raylength));
    return volume->contains(currentPoint);
}

Eigen::Vector3i Raycast::getOriginForInterpolation(const Eigen::Vector3d& point){
    Eigen::Vector3i point_in_grid = point.cast<int>();
    Eigen::Vector3d center = point_in_grid.cast<double>() + Eigen::Vector3d(0.5f,0.5f,0.5f);

    point_in_grid.x() = (point.x() < center.x()) ? (point_in_grid.x() - 1) : point_in_grid.x();
    point_in_grid.y() = (point.y() < center.y()) ? (point_in_grid.y() - 1) : point_in_grid.y();
    point_in_grid.z() = (point.z() < center.z()) ? (point_in_grid.z() - 1) : point_in_grid.z();

    return point_in_grid;
}

double Raycast::getTSDFInterpolation(const Eigen::Vector3d& point,
        const std::shared_ptr<Volume>& volume){
    auto voxelScale = volume->getVoxelScale();
    Eigen::Vector3i origin = getOriginForInterpolation(point);
    Eigen::Vector3d center = origin.cast<double>() + Eigen::Vector3d(0.5f,0.5f,0.5f);

    const double a = (point.x() - center.x());
    const double b = (point.y() - center.y());
    const double c = (point.z() - center.z());

    double tsdf = 0.0f;
    tsdf += volume->getTSDF(Eigen::Vector3d(origin.x()*voxelScale,origin.y()*voxelScale,origin.z()*voxelScale))*(1-a)*(1-b)*(1-c);
    tsdf += volume->getTSDF(Eigen::Vector3d(origin.x()*voxelScale,origin.y()*voxelScale,(origin.z()+1.0f)*voxelScale))*(1-a)*(1-b)*(c);
    tsdf += volume->getTSDF(Eigen::Vector3d(origin.x()*voxelScale,(origin.y()+1.0f)*voxelScale,origin.z()*voxelScale))*(1-a)*(b)*(1-c);
    tsdf += volume->getTSDF(Eigen::Vector3d(origin.x()*voxelScale,(origin.y()+1.0f)*voxelScale,(origin.z()+1.0f)*voxelScale))*(1-a)*(b)*(c);
    tsdf += volume->getTSDF(Eigen::Vector3d((origin.x()+1.0f)*voxelScale,origin.y()*voxelScale,origin.z()*voxelScale))*(a)*(1-b)*(1-c);
    tsdf += volume->getTSDF(Eigen::Vector3d((origin.x()+1.0f)*voxelScale,origin.y()*voxelScale,(origin.z()+1.0f)*voxelScale))*(a)*(1-b)*(c);
    tsdf += volume->getTSDF(Eigen::Vector3d((origin.x()+1.0f)*voxelScale,(origin.y()+1.0f)*voxelScale,origin.z()*voxelScale))*(a)*(b)*(1-c);
    tsdf += volume->getTSDF(Eigen::Vector3d((origin.x()+1.0f)*voxelScale,(origin.y()+1.0f)*voxelScale,(origin.z()+1.0f)*voxelScale))*(a)*(b)*(c);

    return tsdf;
}

Eigen::Vector3d Raycast::calculateNormal(const Eigen::Vector3d& gridVertex,
                                  const std::shared_ptr<Volume>& volume){
    Eigen::Vector3d normal(0.0f,0.0f,0.0f);
    Eigen::Vector3d shiftedVertex;

    shiftedVertex = gridVertex;
    shiftedVertex.x() += 1;
    float tsdf_x1 = getTSDFInterpolation(shiftedVertex,volume);
    shiftedVertex = gridVertex;
    shiftedVertex.x() -= 1;
    float tsdf_x2 = getTSDFInterpolation(shiftedVertex,volume);
    normal.x() = tsdf_x1-tsdf_x2;


    shiftedVertex = gridVertex;
    shiftedVertex.y() += 1;
    float tsdf_y1 = getTSDFInterpolation(shiftedVertex,volume);
    shiftedVertex = gridVertex;
    shiftedVertex.y() -= 1;
    float tsdf_y2 = getTSDFInterpolation(shiftedVertex,volume);
    normal.y() = tsdf_y1-tsdf_y2;


    shiftedVertex = gridVertex;
    shiftedVertex.z() += 1;
    float tsdf_z1 = getTSDFInterpolation(shiftedVertex,volume);
    shiftedVertex = gridVertex;
    shiftedVertex.z() -= 1;
    float tsdf_z2 = getTSDFInterpolation(shiftedVertex,volume);
    normal.z() = tsdf_z1-tsdf_z2;

    return normal;
}

