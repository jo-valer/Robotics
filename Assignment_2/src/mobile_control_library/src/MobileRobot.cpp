#include <iostream>
#include <eigen3/Eigen/Dense>
#include "mobile_control_library/MobileRobot.h"

using namespace std;

MobileRobot::MobileRobot(){
    this->x=0.0;
    this->y=0.0;
    this->th=0.0;
}
MobileRobot::MobileRobot(const double _x, const double _y, const double _th){
    this->x=_x;
    this->y=_y;
    this->th=_th;
}
void MobileRobot::setX(const double _x){
    this->x=_x;
}
void MobileRobot::setY(const double _y){
    this->y=_y;
}
void MobileRobot::setTh(const double _th){
    this->th=_th;
}
double MobileRobot::getX(){
    return x;
}
double MobileRobot::getY(){
    return y;
}
double MobileRobot::getTh(){
    return th;
}