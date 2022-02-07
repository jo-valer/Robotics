#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematic_library/RPYRotation.h"
using namespace Eigen;
using namespace std;

RPYRotation::RPYRotation(){
    this->r = 0.0;
    this->p = 0.0;
    this->y = 0.0;
}
RPYRotation::RPYRotation(const double _r, const double _p, const double _y){
    this->r = _r;
    this->p = _p;
    this->y = _y;
}
void RPYRotation::setR(const double _r){
    this->r = _r;
}
void RPYRotation::setP(const double _p){
    this->p = _p;
}
void RPYRotation::setY(const double _y){
    this->y = _y;
}
double RPYRotation::getR(){
    return r;
}
double RPYRotation::getP(){
    return p;
}
double RPYRotation::getY(){
    return y;
}
