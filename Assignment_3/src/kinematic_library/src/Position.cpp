#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematic_library/Position.h"
using namespace Eigen;
using namespace std;

Position::Position(){
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
}
Position::Position(const double _x, const double _y, const double _z){
    this->x = _x;
    this->y = _y;
    this->z = _z;
}
void Position::setX(const double _x){
    this->x = _x;
}
void Position::setY(const double _y){
    this->y = _y;
}
void Position::setZ(const double _z){
    this->z = _z;
}
double Position::getX(){
    return x;
}
double Position::getY(){
    return y;
}
double Position::getZ(){
    return z;
}
