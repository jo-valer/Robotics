#include "my_world/Coordinate.h"

#include <iostream>

using namespace std;

Coordinate::Coordinate(){
    this->x = 0.0;
    this->y = 0.0;
}
Coordinate::Coordinate(const double _x, const double _y){
    this->x = _x;
    this->y = _y;
}

void Coordinate::setX(const double _x){
    this->x = _x;
}
void Coordinate::setY(const double _y){
    this->y = _y;
}

double Coordinate::getX(){
    return x;
}
double Coordinate::getY(){
    return y;
}