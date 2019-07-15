#include "Fusion.hpp"

Fusion::Fusion(double max_truncation_dist, double min_truncation_dist, double tsdf_max_weight):
_max_truncation_dist(max_truncation_dist),
_min_truncation_dist(min_truncation_dist),
_tsdf_max_weight(tsdf_max_weight)
{}

bool Fusion::reconstructSurface(const std::shared_ptr<Frame>& currentFrame,const std::shared_ptr<Volume>& volume){

    auto volumeSize =volume->getVolumeSize();
    auto voxelScale = volume->getVoxelScale();
    auto pose = currentFrame->getGlobalPose();
    Eigen::Vector3d translation = pose.block(0,3,3,1);
    auto width = currentFrame->getWidth();
    auto depth_map = currentFrame->getDepthMap();

     for (size_t z = 0;z<volumeSize.z();z++) {
		 for( size_t y =0;y<volumeSize.y();y++){
    		for(size_t x=0;x< volumeSize.x();x++){
				/*
				 * Volumetric Reconstruction
				 */
				/*//calculate Camera Position
				Eigen::Vector3d currentCameraPosition;
				Eigen::Vector2i X;*/
				
				Eigen::Vector3d global_point((x+0.5f)*voxelScale,(y+0.5f)*voxelScale,(z+0.5f)*voxelScale);
				Eigen::Vector3d camera_point = currentFrame->projectIntoCamera(global_point);
				Eigen::Vector2i image_point = currentFrame->projectOntoPlane(camera_point);

				if(!currentFrame->contains(image_point))
                    continue;

				double depth_value = depth_map[image_point.y() * width + image_point.x()];

				if (depth_value < 0) continue;

				double current_tsdf = calculateSDF(global_point, translation, depth_value);
                double current_weight = 1.0;
                updateTSDFValue(volume,x,y,z,current_tsdf,current_weight);


				/*if (!calculateGlobal2CameraPoint(currentCameraPosition, x, y, z, pose.block(0,0,3,3), pose.block(0,3,3,1), voxelScale))continue;
				if (!pi(X, currentCameraPosition, currentFrame->getIntrinsics(), width, height))continue;
*/
				/*const double depth = currentFrame->getDepthMap()[X.y() + (X.x() * width)];
				if (depth <= 0) continue;

				auto lambda = calculateLamdas(X, currentFrame->getIntrinsics());

				auto sdf = calculateSDF(lambda, currentCameraPosition, depth);
*/
				/*
				 * Volumetric Integration
				 */
				/*if (sdf >= _min_truncation_dist) {
					if(sdf<0)throw "Check why sdf is negative / in paper they use the sign function";
					const double current_tsdf = std::min(1., sdf / _max_truncation_dist); // *sgn(sdf)

					// Improvement(Parika) : weight can be set proportional to cosine of the angle
					// between surface normal and measurement ray
					double current_weight = 1.0;
                    updateTSDFValue(volume,x,y,z,current_tsdf,current_weight);
				}*/
			}
        }
    }
    return true;
}

/*bool Fusion::calculateGlobal2CameraPoint(Eigen::Vector3d &currentCameraPosition, int x, int y, int z,
										 const Eigen::Matrix<double, 3, 3>& rotation,
										 const Eigen::Vector3d& translation, double voxelScale){


    const Eigen::Vector3d position((static_cast<double>(x) + 0.5) * voxelScale,
                           (static_cast<double>(y) + 0.5) * voxelScale,
                           (static_cast<double>(z) + 0.5) * voxelScale);
    currentCameraPosition = rotation * position + translation;

    //TODO: verify why this line makes sense
    if (currentCameraPosition.z() <= 0) return false;


    return true;
}
bool Fusion::pi(Eigen::Vector2i& pi,const Eigen::Vector3d& currentPos, const Eigen::Matrix3d& intrinsics,int width, int height){
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
}*/

/*double Fusion::calculateLamdas(Eigen::Vector2i &cameraSpacePoint,const Eigen::Matrix3d& intrinsics) {
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
	//TODO: check if cameraPosition should actually be only translation-p
    return (-1.f) * ((1.f / lambda) * cameraPosition.norm() - rawDepthValue);
}*/

double Fusion::calculateSDF(Eigen::Vector3d& globalPoint ,
        Eigen::Vector3d& translation,
        double rawDepthValue){
    double sdf = (translation - globalPoint).norm() - rawDepthValue;

    if (sdf > 0){
        sdf = std::min(1.0, sdf/_max_truncation_dist);
    }
    else{
        sdf = std::max(-1.0, sdf/_min_truncation_dist);
    }

    return sdf;
}

void Fusion::updateTSDFValue(const std::shared_ptr<Volume>& volume,
        size_t x, size_t y, size_t z,
        double current_tsdf,
        double current_weight){

    std::pair<double,double> voxelData= volume->getVoxelData(x,y,z);
    const double tsdf=voxelData.first;
    const double weight = voxelData.second;

    double updated_weight = weight+current_weight;
    if (updated_weight > _tsdf_max_weight)
        updated_weight = _tsdf_max_weight;

    double updated_tsdf = (weight*tsdf + current_weight*current_tsdf)/updated_weight;

    volume->updateVoxelData(x,y,z,updated_tsdf,updated_weight);
}


