#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematic_library/Ur5JointConfiguration.h"
using namespace Eigen;
using namespace std;

Ur5JointConfiguration::Ur5JointConfiguration(){
    this->th1=0.0;
    this->th2=0.0;
    this->th3=0.0;
    this->th4=0.0;
    this->th5=0.0;
    this->th6=0.0;
}
Ur5JointConfiguration::Ur5JointConfiguration(const double _th1,const double _th2,const double _th3,const double _th4,const double _th5,const double _th6){
    this->th1=_th1;
    this->th2=_th2;
    this->th3=_th3;
    this->th4=_th4;
    this->th5=_th5;
    this->th6=_th6;
}
void Ur5JointConfiguration::setAngles(const double jntAngles[6]){
    th1=jntAngles[0];
    th2=jntAngles[1];
    th3=jntAngles[2];
    th4=jntAngles[3];
    th5=jntAngles[4];
    th6=jntAngles[5];
}
void Ur5JointConfiguration::setTh1(const double _th){
    th1 = _th;
}
void Ur5JointConfiguration::setTh2(const double _th){
    th2 = _th;
}
void Ur5JointConfiguration::setTh3(const double _th){
    th3 = _th;
}
void Ur5JointConfiguration::setTh4(const double _th){
    th4 = _th;
}
void Ur5JointConfiguration::setTh5(const double _th){
    th5 = _th;
}
void Ur5JointConfiguration::setTh6(const double _th){
    th6 = _th;
}
double Ur5JointConfiguration::getTh1(){
    return th1;
}
double Ur5JointConfiguration::getTh2(){
    return th2;
}
double Ur5JointConfiguration::getTh3(){
    return th3;
}
double Ur5JointConfiguration::getTh4(){
    return th4;
}
double Ur5JointConfiguration::getTh5(){
    return th5;
}
double Ur5JointConfiguration::getTh6(){
    return th6;
}
