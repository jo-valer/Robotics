#ifndef MOBILEROBOT_H
#define MOBILEROBOT_H

class MobileRobot{
    private:
        double x;
        double y;
        double th;

    public:
        MobileRobot();
        MobileRobot(const double _x, const double _y, const double _th);
        void setX(const double _x);
        void setY(const double _y);
        void setTh(const double _th);
        double getX();
        double getY();
        double getTh();
};
#endif
