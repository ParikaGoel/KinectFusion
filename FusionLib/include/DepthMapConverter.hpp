//
// Created by pbo on 27.06.19.
//
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <Utils.hpp>
#include <EigenHelper.hpp>

#include <sophus/se3.hpp>

class DepthMapConverter {
public:
	DepthMapConverter(std::shared_ptr<std::vector<Point2D>> imageData, int imageWidth, int imageHeight, Eigen::Matrix3d
	intrinsics, Eigen::Matrix4d extrinsics);

	void ImageToWorld();

	/*
	 * This Method assumes, that _camera Vertex contains the vertices in row-major order,
	 * Therefore parallelization isnt possible as the order might be changed
	 */
	void computeNormals();


private:
	std::shared_ptr<std::vector<Point2D>> _imageData;
	std::shared_ptr<std::vector<Vector3d>> _cameraVertex;
	std::shared_ptr<std::vector<Vector3d>> _globalVertex;
	std::shared_ptr<std::vector<Vector3d>> _cameraNormals;
	std::shared_ptr<std::vector<Vector3d>> _globalNormals;
	Eigen::Matrix3d _intrinsics;
	Sophus::SE3d _extrinsics;
	int _width, _height;

	/*
	 *
	 * Getters & Setters
	 *
	 */
public:

	const std::shared_ptr<std::vector<Vector3d>> &getCameraVertexPtr() const;

	const std::shared_ptr<std::vector<Vector3d>> &getGlobalVertexPtr() const;

	const std::shared_ptr<std::vector<Vector3d>> &getCameraNormalPtr() const;

	const std::shared_ptr<std::vector<Vector3d>> &getGlobalNormalsPtr() const;

	const std::vector<Vector3d> &getCameraVertex() const;

	const std::vector<Vector3d> &getGlobalVertex() const;

	const std::vector<Vector3d> &getCameraNormal() const;

	const std::vector<Vector3d> &getGlobalNormals() const;

	const Matrix3d &getIntrinsics() const;

	const Sophus::SE3d &getGlobalPose() const;

	const std::vector<double> &getDepthMap() const;

	const double getWidth();

	const double getHeight();

};