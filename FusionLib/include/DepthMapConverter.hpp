//
// Created by pbo on 27.06.19.
//
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <Utils.hpp>
#include <EigenHelper.hpp>

class DepthMapConverter{
public:
    DepthMapConverter(std::shared_ptr<std::vector<Point2D>> imageData,int imageWidth,int imageHeight, Eigen::Matrix3f intrinsics, Eigen::Matrix4f extrinsics);
    void ImageToWorld();




private:
    std::shared_ptr<std::vector<Point2D>> _imageData;
    std::shared_ptr<std::vector<Vector3f>> _cameraVertex;
    std::shared_ptr<std::vector<Vector4f>> _globalVertex;
    Eigen::Matrix3f _intrinsics;
    Eigen::Matrix4f _extrinsics;
    int _width,_height;

    /*
     *
     * Getters & Setters
     *
     */
public:

    const std::shared_ptr<std::vector<Vector3f>> &getCameraVertex() const;

    const std::shared_ptr<std::vector<Vector4f>> &getGlobalVertex() const;

};