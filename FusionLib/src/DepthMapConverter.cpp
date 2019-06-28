#include "DepthMapConverter.hpp"



DepthMapConverter::DepthMapConverter(std::shared_ptr<std::vector<Point2D>> imageData,int imageWidth,int imageHeight, Eigen::Matrix3f intrinsics,
                                     Eigen::Matrix4f extrinsics):
                                     _imageData(imageData),
                                     _intrinsics(intrinsics),
                                     _extrinsics(extrinsics),
                                     _width(imageWidth),
                                     _height(imageHeight){

    _worldData = std::make_shared<std::vector<Vector4f>>(imageData->size());
    ImageToWorld();

}

void DepthMapConverter::ImageToWorld() {
    auto intrinsicsInverse = _intrinsics.inverse();
    auto trajectoryInv = _extrinsics.inverse();
    for(Point2D vertex :*_imageData){
        Vector4f worldPoint(0,0,0,1);

        if(vertex._data == MINF){
            worldPoint = Vector4f(MINF,MINF,MINF,MINF);
        }else{
            //calc depthvalue*K⁻1*[u,1]
            Vector3f tmp =vertex._data*intrinsicsInverse*vertex._position;
            worldPoint.block(0,0,3,1) = tmp;
            worldPoint = trajectoryInv * worldPoint;
        }
        _worldData->push_back(worldPoint);
    }
}

const std::shared_ptr<std::vector<Point2D>> &DepthMapConverter::getImageData() const {
    return _imageData;
}

const std::shared_ptr<std::vector<Vector4f>> &DepthMapConverter::getWorldData() const {
    return _worldData;
}

const Eigen::Matrix3f DepthMapConverter::getIntrinsics() const {
    return _intrinsics;
}

const Eigen::Matrix4f DepthMapConverter::getExtrinsics() const {
    return _extrinsics;
}
