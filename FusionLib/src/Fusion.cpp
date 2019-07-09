//
// Created by pbo on 08.07.19.
//

#include "Fusion.hpp"


bool Fusion::reconstructSurface(std::shared_ptr<Frame> currentFrame,std::shared_ptr<Volume> volume,double truncationDistance){

    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose();
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();
    //TODO check if row or col major order
    for(int x=0;x< volumeSize.x();x++){
        for( int y =0;y<volumeSize.y();y++){
            for (int z = 0;z<volumeSize.z();z++){

                /*
                 * Volumetric Reconstruction
                 */
                //calculate Camera Position
                Eigen::Vector3d currentCameraPosition;
                Eigen::Vector2i X;
                if(!calculateCurrentCameraPosition(currentCameraPosition,x, y, z, pose.rotationMatrix(), pose.translation(), voxelScale))continue;
                if(!pi(X,currentCameraPosition,currentFrame->getIntrinsics(),width,height))continue;

                const double depth = currentFrame->getRawDepthMap()[X.y()+(X.x()*width)];
                if (depth <= 0) continue;

                auto lambda=calculateLamdas(X, currentFrame->getIntrinsics());

                auto sdf=calculateSDF(lambda,currentCameraPosition,depth);

                /*
                 * Volumetric Integration
                 */
                if (sdf >= -truncationDistance) {
                    //TODO implement integration using the tsdfs stored in VOLUME



                }

        }
    }
    return true;
}
//TODO: change return type
void Fusion::reconstruct(std::shared_ptr<Frame> currentFrame,std::shared_ptr<Volume> volume,float truncationDistance){
    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose();
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();
    //TODO check if row or col major order
    for(int x=0;x< volumeSize.x();x++){
        for( int y =0;y<volumeSize.y();y++){
            for (int z = 0;z<volumeSize.z();z++){


                //calculate Camera Position
                Eigen::Vector3d currentCameraPosition;
                Eigen::Vector2i X;
                if(!calculateCurrentCameraPosition(currentCameraPosition,x, y, z, pose.rotationMatrix(), pose.translation(), voxelScale))continue;
                if(!pi(X,currentCameraPosition,currentFrame->getIntrinsics(),width,height))continue;

                const double depth = currentFrame->getRawDepthMap()[X.y()+(X.x()*width)];
                if (depth <= 0) continue;

                auto lambda=calculateLamdas(X, currentFrame->getIntrinsics());

                calculateSDF(lambda,currentCameraPosition,depth);





            }

        }
    }

}

bool Fusion::calculateCurrentCameraPosition(Eigen::Vector3d& currentCameraPosition,int x, int y, int z,Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation,
                                   Eigen::Vector3d translation, double voxelScale){


    const Eigen::Vector3d position((static_cast<double>(x) + 0.5) * voxelScale,
                           (static_cast<double>(y) + 0.5) * voxelScale,
                           (static_cast<double>(z) + 0.5) * voxelScale);
    //x = KT⁻¹p
    currentCameraPosition = rotation * position + translation;

    //TODO: verify why this line makes sense
    if (currentCameraPosition.z() <= 0) return false;


    return true;
}
bool Fusion::pi(Eigen::Vector2i& pi,Eigen::Vector3d currentPos, Eigen::Matrix3d intrinsics,int width, int height){
    double fovX = intrinsics(0, 0);
    double fovY = intrinsics(1, 1);
    double cX = intrinsics(0, 2);
    double cY = intrinsics(1, 2);
    //X=pi(x)

    pi = Eigen::Vector2i(
            (int)(currentPos.x() / currentPos.z() * fovX + cX),
            (int)(currentPos.y() / currentPos.z() * fovY + cY)
    );
    //TODO: Check if x coordinate actually should be compared to getWdith() or actually to getHeight()

    if (pi.x() < 0 || pi.x() >= width || pi.y() < 0 || pi.y() >= height)
        return false;

    return true;
}

double Fusion::calculateLamdas(Eigen::Vector2i &cameraSpacePoint,Eigen::Matrix3d intrinsics) {
    double fovX = intrinsics(0, 0);
    double fovY = intrinsics(1, 1);
    double cX = intrinsics(0, 2);
    double cY = intrinsics(1, 2);
    const Eigen::Vector2i lambda(
            (cameraSpacePoint.x() - cX) / fovX,
            (cameraSpacePoint.y() - cY) / fovY,
            1.f);

    return lambda.norm();
}

double Fusion::calculateSDF(double &lambda, Eigen::Vector3d &cameraPosition, double rawDepthValue) {
    return (-1.f) * ((1.f / lambda) * cameraPosition.norm() - rawDepthValue);
}


