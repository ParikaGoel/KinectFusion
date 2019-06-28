//
// Created by Philipp Bock on 28.06.19.
//
#pragma once


class Point2D{
public:

    Point2D():_data(0),_position(0,0,1){}
    Point2D(int x,int y, float data):_data(data),_position(x,y,1),xx(x),yy(y){}
    Eigen::Vector3f _position;
    float _data;
    int xx,yy;
};

