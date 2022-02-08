#ifndef UR5DH_H
#define UR5DH_H
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

class Ur5DH{
    private:
        double A[6];
        double D[6];
    
    public:
        Ur5DH(double A[6], double D[6]);
};

#endif
