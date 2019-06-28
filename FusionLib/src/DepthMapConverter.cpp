#include "DepthMapConverter.hpp"



DepthMapConverter::DepthMapConverter(std::shared_ptr<std::vector<Point2D>> imageData,int imageWidth,int imageHeight, Eigen::Matrix3f intrinsics,
                                     Eigen::Matrix4f extrinsics):
                                     _imageData(imageData),
                                     _intrinsics(intrinsics),
                                     _extrinsics(extrinsics),
                                     _width(imageWidth),
                                     _height(imageHeight){

    _globalVertex = std::make_shared<std::vector<Vector4f>>(imageData->size());
    _cameraVertex = std::make_shared<std::vector<Vector3f>>(imageData->size());

    ImageToWorld();

}

void DepthMapConverter::ImageToWorld() {
    auto intrinsicsInverse = _intrinsics.inverse();
    auto trajectoryInv = _extrinsics.inverse();
    for(Point2D vertex :*_imageData){
        Vector4f worldVertex(0,0,0,1);
        Vector3f cameraVertex;

        if(vertex._data == MINF){
            worldVertex = Vector4f(MINF,MINF,MINF,MINF);
        }else{
            //calc depthvalue*K⁻1*[u,1]
            Vector3f cameraVertex =vertex._data*intrinsicsInverse*vertex._position;
            worldVertex.block(0,0,3,1) = cameraVertex;
            worldVertex = trajectoryInv * worldVertex;
        }
        _cameraVertex->push_back(cameraVertex);
        _globalVertex->push_back(worldVertex);

    }
}

const std::shared_ptr<std::vector<Vector3f>> &DepthMapConverter::getCameraVertex() const {
    return _cameraVertex;
}

const std::shared_ptr<std::vector<Vector4f>> &DepthMapConverter::getGlobalVertex() const {
    return _globalVertex;
}

