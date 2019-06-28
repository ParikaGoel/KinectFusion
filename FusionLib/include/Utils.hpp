//
// Created by pbo on 28.06.19.
//
#pragma once


struct Point2D{
    Point2D():_data(0),_position(0,0,1){}
    Point2D(int x,int y, float data):_data(data),_position(x,y,1){}
    Eigen::Vector3f _position;
    float _data;
};

