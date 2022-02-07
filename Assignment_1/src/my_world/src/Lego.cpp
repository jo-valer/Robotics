#include <iostream>
#include "my_world/Lego.h"

using namespace std;

Lego::Lego(){
    this->name="";
    this->x=0.0;
    this->y=0.0;
    this->classe=0;
}
Lego::Lego(string _name, const double _x, const double _y, const int _c){
    this->name=_name;
    this->x=_x;
    this->y=_y;
    this->classe=_c;
}
void Lego::setName(string _name){
    this->name=_name;
}
void Lego::setX(const double _x){
    this->x=_x;
}
void Lego::setY(const double _y){
    this->y=_y;
}
void Lego::setClasse(const int _c){
    this->classe=_c;
}
string Lego::getName(){
    return name;
}
double Lego::getX(){
    return x;
}
double Lego::getY(){
    return y;
}
int Lego::getClasse(){
    return classe;
}
double Lego::getGrip(){
    double grip;
    switch (this->classe){
        case 0: grip = 0.4; break;
        case 1: grip = 0.4; break;
        case 2: grip = 0.4; break;
        case 3: grip = 0.4; break;
        case 4: grip = 0.4; break;
        case 5: grip = 0.4; break;
        case 6: grip = 0.4; break;
        case 7: grip = 0.4; break;
        case 8: grip = 0.4; break;
        case 9: grip = 0.1; break;
        case 10: grip = 0.1; break;
        default: grip = 0.1;
    }
    return grip;
}
double Lego::getZapproach(){
    double z;
    switch (this->classe){
        case 0: z = -0.11; break;
        case 1: z = -0.14; break;
        case 2: z = -0.11; break;
        case 3: z = -0.11; break;
        case 4: z = -0.11; break;
        case 5: z = -0.11; break;
        case 6: z = -0.11; break;
        case 7: z = -0.14; break;
        case 8: z = -0.11; break;
        case 9: z = -0.11; break;
        case 10: z = -0.11; break;
        default: z = -0.11;
    }
    return z;
}

