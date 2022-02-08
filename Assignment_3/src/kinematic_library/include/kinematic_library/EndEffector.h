#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "Position.h"
#include "RPYRotation.h"

using namespace Eigen;

class EndEffector{
    private:
        Position position;
        RPYRotation orientation;
        Matrix3d rotationMatrix;

    public:
        EndEffector();
        EndEffector(const double _x, const double _y, const double _z, const double _roll, const double _pitch, const double _yaw);
        void setPostion(const double _x, const double _y, const double _z);
        void setOrientation(const double _roll, const double _pitch, const double _yaw);
        void setRotationMatrix(Matrix3d& m);
        Position getPosition();
        RPYRotation getOrientation();
        Matrix3d getRotationMatrix();
};
#endif
