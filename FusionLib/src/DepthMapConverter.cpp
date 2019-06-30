
//
// Created by Philipp Bock on 28.06.19.
//

#include "DepthMapConverter.hpp"


DepthMapConverter::DepthMapConverter(std::shared_ptr<std::vector<Point2D>> imageData, int imageWidth, int imageHeight,
									 Eigen::Matrix3d intrinsics, Eigen::Matrix4d extrinsics)
		:
		_imageData(imageData)
		, _intrinsics(intrinsics)
		, _width(imageWidth)
		, _height(imageHeight) {

	_globalVertex = std::make_shared<std::vector<Vector3d>>(imageData->size());
	_cameraVertex = std::make_shared<std::vector<Vector3d>>(imageData->size());
	_globalNormals = std::make_shared<std::vector<Vector3d>>(imageData->size());
	_cameraNormals = std::make_shared<std::vector<Vector3d>>(imageData->size());

	_extrinsics.setRotationMatrix(extrinsics.block<3,3>(0,0));
	_extrinsics.transX(extrinsics(0,3));
    _extrinsics.transX(extrinsics(1,3));
    _extrinsics.transX(extrinsics(2,3));

	ImageToWorld();
	computeNormals();

}

void DepthMapConverter::ImageToWorld() {
	auto intrinsicsInverse = _intrinsics.inverse();
	auto trajectoryInv = _extrinsics.inverse();
	for (Point2D vertex :*_imageData) {
		Vector3d worldVertex(0, 0, 0);
		Vector3d cameraVertex;

		if (vertex._data == MINF) {
			worldVertex = Vector3d(MINF, MINF, MINF);
		} else {
			//calc depthvalue*K⁻1*[u,1]
			Vector3d cameraVertex = vertex._data * intrinsicsInverse * vertex._position;
			worldVertex = trajectoryInv * cameraVertex;
		}
		_cameraVertex->push_back(cameraVertex);
		_globalVertex->push_back(worldVertex);

	}
}

void DepthMapConverter::computeNormals() {
	auto trajectoryInv = _extrinsics.inverse();
	Vector3d globalNormal(0, 0, 0);
	for (int y = 0; y < _height; y++) {
		for (int x = 0; x < _width; x++) {
			/*
			 *
			 * Debugging Variables:
			 *
			int xx = _imageData->at(y*_width+x)._position.x();
			int yy = _imageData->at(y*_width+x)._position.y();

			 */

			if (_imageData->at(y * _width + x)._position.x() != x ||
				_imageData->at(y * _width + x)._position.y() != y) {
				throw std::invalid_argument(" The order of _imageData is not correct (e.g. not Row-major or parrallelization applied) ");
			}


			Vector3d v1 = _cameraVertex->at(y * _width + x + 1) - _cameraVertex->at(y * _width + x);
			Vector3d v2 = _cameraVertex->at((y + 1) * _width + x) - _cameraVertex->at(y * _width + x);
			auto normal = v1.cross(v2);
			normal.normalize();
			_cameraNormals->push_back(normal);
			globalNormal = trajectoryInv * normal;
			_globalNormals->push_back(globalNormal);
		}
	}
}

const std::shared_ptr<std::vector<Vector3d>> &DepthMapConverter::getCameraVertexPtr() const {
	return _cameraVertex;
}

const std::shared_ptr<std::vector<Vector3d>> &DepthMapConverter::getGlobalVertexPtr() const {
	return _globalVertex;
}

const std::shared_ptr<std::vector<Vector3d>> &DepthMapConverter::getCameraNormalPtr() const {
	return _cameraNormals;
}

const std::shared_ptr<std::vector<Vector3d>> &DepthMapConverter::getGlobalNormalsPtr() const {
	return _globalNormals;
}

const std::vector<Vector3d> &DepthMapConverter::getCameraVertex() const {
	return *_cameraVertex;
}

const std::vector<Vector3d> &DepthMapConverter::getGlobalVertex() const {
	return *_globalVertex;
}

const std::vector<Vector3d> &DepthMapConverter::getCameraNormal() const {
	return *_cameraNormals;
}

const std::vector<Vector3d> &DepthMapConverter::getGlobalNormals() const {
	return *_globalNormals;
}

const Matrix3d &DepthMapConverter::getIntrinsics() const{
    return _intrinsics;
}

const Sophus::SE3d &DepthMapConverter::getGlobalPose() const{
    return _extrinsics;
}

// ToDo: Require to return the depth map
// Depth Map : 2D array of depth values
const std::vector<double> &DepthMapConverter::getDepthMap() const{
    std::vector<double> temp;
    return temp;
}

const double DepthMapConverter::getWidth(){
    return _width;
}

const double DepthMapConverter::getHeight(){
    return _height;
}

