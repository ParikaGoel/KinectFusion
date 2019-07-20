#include <iostream>
#include <MeshWriter.h>
#include "Fusion.hpp"


bool Fusion::reconstructSurface(const std::shared_ptr<Frame>& currentFrame,const std::shared_ptr<Volume>& volume,double truncationDistance){

    auto volumeSize =volume->getVolumeSize();
    auto pose = currentFrame->getGlobalPose().inverse();
    auto width = currentFrame->getWidth();
    auto voxelData = volume->getVoxelData();

    Eigen::Matrix3d rotation    = pose.block(0,0,3,3);
    Eigen::Vector3d translation = pose.block(0,3,3,1);

     for (int z = 0;z<volumeSize.z();z++) {
		 for( int y =0;y<volumeSize.y();y++){
    		for(int x=0;x< volumeSize.x();x++){
				/*
				 * Volumetric Reconstruction
				 */
				//calculate Camera Position
				Eigen::Vector3d globalCoord_voxel = volume->getGlobalCoordinate(x, y, z);
                Eigen::Vector3d currentCameraPosition = rotation * globalCoord_voxel + translation;
                if (currentCameraPosition.z() <= 0) continue;

                Eigen::Vector2i img_coord = currentFrame -> projectOntoPlane(currentCameraPosition);

				if (!currentFrame->contains(img_coord))continue;

				const double depth = currentFrame->getDepthMap()[img_coord.x() + (img_coord.y() * width)];
				if (depth <= 0) continue;

				auto lambda = calculateLamdas(img_coord, currentFrame->getIntrinsics());
				auto sdf = calculateSDF(lambda, currentCameraPosition, depth);

				/*
				 * Volumetric Integration
				 */
				if (sdf >= -truncationDistance) {

				    const double current_tsdf = std::min(1., sdf / truncationDistance); // *sgn(sdf)
				    const double current_weight = 1.0;
					size_t voxel_index = x+(y*volumeSize.x())+(z*volumeSize.x()*volumeSize.y());
					const double old_tsdf=volume->getVoxelData()[voxel_index].tsdf;
					const double old_weight = volume->getVoxelData()[voxel_index].weight;

					const double updated_tsdf = (old_weight*old_tsdf + current_weight*current_tsdf)/
							(old_weight+current_weight);
					const double updated_weight = old_weight+current_weight;

                    volume->getVoxelData()[voxel_index].tsdf = updated_tsdf;
                    volume->getVoxelData()[voxel_index].weight = updated_weight;

                    if (sdf <= truncationDistance / 2 && sdf >= -truncationDistance / 2) {
                        Vector4uc& voxel_color = voxelData[voxel_index].color;
                        const Vector4uc image_color = currentFrame->getColorMap()[img_coord.x() + (img_coord.y() * width)];

                        voxel_color[0] = (old_weight * voxel_color[0] + current_weight * image_color[0]) /
                                (old_weight + current_weight);
                        voxel_color[1] = (old_weight * voxel_color[1] + current_weight * image_color[1]) /
                                (old_weight + current_weight);
                        voxel_color[2] =(old_weight * voxel_color[2] + current_weight * image_color[2]) /
                                (old_weight + current_weight);
                        voxel_color[3] =(old_weight * voxel_color[3] + current_weight * image_color[3]) /
                                        (old_weight + current_weight);
                    }

				}
			}
        }
    }
     MeshWriter::toFile(std::string("tsdf"), *volume, 1, 0.7);
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


