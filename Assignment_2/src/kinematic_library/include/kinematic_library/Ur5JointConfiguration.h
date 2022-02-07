#ifndef UR5JOINTCONFIGURATION_H
#define UR5JOINTCONFIGURATION_H
#include <ros/ros.h>
class Ur5JointConfiguration{
    private:
        double th1;
        double th2;
        double th3;
        double th4;
        double th5;
        double th6;
    public:
        Ur5JointConfiguration();
        Ur5JointConfiguration(const double _th1,const double _th2,const double _th3,const double _th4,const double _th5,const double _th6);
        void setAngles(const double jntAngles[6]);
        void setTh1(const double _th);
        void setTh2(const double _th);
        void setTh3(const double _th);
        void setTh4(const double _th);
        void setTh5(const double _th);
        void setTh6(const double _th);
        double getTh1();
        double getTh2();
        double getTh3();
        double getTh4();
        double getTh5();
        double getTh6();
};

#endif
