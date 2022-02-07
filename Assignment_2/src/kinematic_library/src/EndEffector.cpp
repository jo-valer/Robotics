#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematic_library/EndEffector.h"
using namespace Eigen;
using namespace std;

EndEffector::EndEffector(){
    position.setX(0.0);
    position.setY(0.0);
    position.setZ(0.0);
    orientation.setR(0.0);
    orientation.setP(0.0);
    orientation.setY(0.0);
}
EndEffector::EndEffector(const double _x, const double _y, const double _z, const double _roll, const double _pitch, const double _yaw){
    position.setX(_x);
    position.setY(_y);
    position.setZ(_z);
    orientation.setR(_roll);
    orientation.setP(_pitch);
    orientation.setY(_yaw);
}
void EndEffector::setPostion(const double _x, const double _y, const double _z){
    position.setX(_x);
    position.setY(_y);
    position.setZ(_z);
}
void EndEffector::setOrientation(const double _roll, const double _pitch, const double _yaw){
    orientation.setR(_roll);
    orientation.setP(_pitch);
    orientation.setY(_yaw);
}
void EndEffector::setRotationMatrix(Matrix3d& m){
    this->rotationMatrix = m;
}
Position EndEffector::getPosition(){
    return position;
}
RPYRotation EndEffector::getOrientation(){
    return orientation;
}
Matrix3d EndEffector::getRotationMatrix(){
    return rotationMatrix;
}
