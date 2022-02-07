#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematic_library/Ur5DH.h"
using namespace Eigen;
using namespace std;

Ur5DH::Ur5DH(double A[6], double D[6]){
    for(int i=0; i<6; i++){
        this->A[i] = A[i];
        this->D[i] = D[i];
    }
}
