#ifndef POSITION_H
#define POSITION_H
#include <ros/ros.h>
class Position{
    private:
        double x;
        double y;
        double z; 
    
    public:
        Position();
        Position(const double _x, const double _y, const double _z);
        void setX(const double _x);
        void setY(const double _y);
        void setZ(const double _z);
        double getX();
        double getY();
        double getZ();
};

#endif
