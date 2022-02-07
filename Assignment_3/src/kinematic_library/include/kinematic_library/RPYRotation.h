#ifndef RPYRotation_H
#define RPYRotation_H
#include <ros/ros.h>
class RPYRotation{
    private:
        double r;
        double p;
        double y; 
    
    public:
        RPYRotation();
        RPYRotation(const double _r, const double _p, const double _y);
        void setR(const double _r);
        void setP(const double _p);
        void setY(const double _y);
        double getR();
        double getP();
        double getY();
};

#endif
