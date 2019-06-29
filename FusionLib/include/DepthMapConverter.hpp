//
// Created by pbo on 27.06.19.
//
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <Utils.hpp>
#include <EigenHelper.hpp>

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
	std::shared_ptr<std::vector<Vector4d>> _globalVertex;
	std::shared_ptr<std::vector<Vector3d>> _cameraNormals;
	std::shared_ptr<std::vector<Vector4d>> _globalNormals;
	Eigen::Matrix3d _intrinsics;
	Eigen::Matrix4d _extrinsics;
	int _width, _height;

	/*
	 *
	 * Getters & Setters
	 *
	 */
public:

	const std::shared_ptr<std::vector<Vector3d>> &getCameraVertexPtr() const;

	const std::shared_ptr<std::vector<Vector4d>> &getGlobalVertexPtr() const;

	const std::shared_ptr<std::vector<Vector3d>> &getCameraNormalPtr() const;

	const std::shared_ptr<std::vector<Vector4d>> &getGlobalNormalsPtr() const;

	const std::vector<Vector3d> &getCameraVertex() const;

	const std::vector<Vector4d> &getGlobalVertex() const;

	const std::vector<Vector3d> &getCameraNormal() const;

	const std::vector<Vector4d> &getGlobalNormals() const;

};