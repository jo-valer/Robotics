#ifndef COORDINATE_H
#define COORDINATE_H

class Coordinate{
    private:
        double x;
        double y;

    public:
        Coordinate();
        Coordinate(const double _x, const double _y);
        
        void setX(const double _x);
        void setY(const double _y);

        double getX();
        double getY();
};

#endif

