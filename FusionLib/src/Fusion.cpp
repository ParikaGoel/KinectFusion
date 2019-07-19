#include "Fusion.hpp"


bool Fusion::reconstructSurface(const std::shared_ptr<Frame>& currentFrame,const std::shared_ptr<Volume>& volume,double truncationDistance){

    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose().inverse();
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();
    auto tsdfVolumeData = volume->getVoxelData();

     for (int z = 0;z<volumeSize.z();z++) {
		 for( int y =0;y<volumeSize.y();y++){
    		for(int x=0;x< volumeSize.x();x++){
				/*
				 * Volumetric Reconstruction
				 */
				//calculate Camera Position

                Eigen::Vector3d currentCameraPosition;
                Eigen::Vector2i X;
                if (!calculateGlobal2CameraPoint(currentCameraPosition, x, y, z, pose.block(0,0,3,3), pose.block(0,3,3,1), voxelScale))continue;

                currentCameraPosition += volume->getOrigin();

				if (!pi(X, currentCameraPosition, currentFrame->getIntrinsics(), width, height))continue;

				const double depth = currentFrame->getDepthMap()[X.x() + (X.y() * width)];
				if (depth <= 0) continue;

				auto lambda = calculateLamdas(X, currentFrame->getIntrinsics());

				auto sdf = calculateSDF(lambda, currentCameraPosition, depth);

				/*
				 * Volumetric Integration
				 */
				if (sdf >= -truncationDistance) {

				    const double current_tsdf = std::min(1., sdf / truncationDistance); // *sgn(sdf)
					const double current_weight = 1.0;
					size_t tsdf_index = x+(y*volumeSize.x())+(z*volumeSize.x()*volumeSize.y());
					const double old_tsdf=tsdfVolumeData[tsdf_index].tsdf;
					const double old_weight = tsdfVolumeData[tsdf_index].weight;

					const double updated_tsdf = (old_weight*old_tsdf + current_weight*current_tsdf)/
							(old_weight+current_weight);
					const double updated_weight = old_weight+current_weight;

                    tsdfVolumeData[tsdf_index].tsdf = updated_tsdf;
                    tsdfVolumeData[tsdf_index].weight = updated_weight;

				}
			}
        }
    }
    return true;
}

bool Fusion::calculateGlobal2CameraPoint(Eigen::Vector3d &currentCameraPosition, int x, int y, int z,
										 const Eigen::Matrix3d& rotation,
										 const Eigen::Vector3d& translation,
										 double voxelScale){


    const Eigen::Vector3d position((static_cast<double>(x) + 0.5) * voxelScale,
                           (static_cast<double>(y) + 0.5) * voxelScale,
                           (static_cast<double>(z) + 0.5) * voxelScale);
    currentCameraPosition = rotation * position + translation;

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

    if (pi.x() < 0 || pi.x() >= width || pi.y() < 0 || pi.y() >= height)
        return false;

    return true;
}

double Fusion::calculateLamdas(Eigen::Vector2i &cameraSpacePoint,Eigen::Matrix3d intrinsics) {
    double fovX = intrinsics(0, 0);
    double fovY = intrinsics(1, 1);
    double cX = intrinsics(0, 2);
    double cY = intrinsics(1, 2);
    const Eigen::Vector3d lambda(
            (cameraSpacePoint.x() - cX) / fovX,
            (cameraSpacePoint.y() - cY) / fovY,
            1.);

    return lambda.norm();
}

double Fusion::calculateSDF(double &lambda, Eigen::Vector3d &cameraPosition, double rawDepthValue) {
    return (-1.f) * ((1.f / lambda) * cameraPosition.norm() - rawDepthValue);
}


