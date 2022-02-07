#include <iostream>
#include "my_world/Lego.h"

using namespace std;

Lego::Lego(){
    this->name="";
    this->x=0.0;
    this->y=0.0;
    this->classe=0;
    this->orientation=0.0;
    this->pose=0;
}
Lego::Lego(string _name, const double _x, const double _y, const int _classe, const double _o, const int _p){
    this->name=_name;
    this->x=_x;
    this->y=_y;
    this->classe=_classe;
    this->orientation=_o;
    this->pose=_p;
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
void Lego::setOrientation(const double _o){
    this->orientation=_o;
}
void Lego::setPose(const int _p){
    this->pose=_p;
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
double Lego::getOrientation(){
    return orientation;
}
int Lego::getPose(){
    return pose;
}
double Lego::getGrip(){
    double grip;
    switch (this->classe){
        case 0:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 1:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 2:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 3:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 4:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 5:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 6:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 7:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 8:  if(this->pose == 0){grip = 0.4;}else{grip = 0.3;} break;
        case 9:  if(this->pose == 0){grip = 0.1;}else{grip = 0.3;} break;
        case 10: if(this->pose == 0){grip = 0.1;}else{grip = 0.3;} break;
        default: grip = 0.1;
    }
    return grip;
}
double Lego::getZapproach(){
    double z;
    switch (this->classe){
        case 0:  z = -0.11; break;
        case 1:  z = -0.14; break;
        case 2:  z = -0.11; break;
        case 3:  z = -0.11; break;
        case 4:  z = -0.11; break;
        case 5:  z = -0.11; break;
        case 6:  z = -0.11; break;
        case 7:  z = -0.14; break;
        case 8:  z = -0.11; break;
        case 9:  if(this->pose == 0){z = -0.11;}else{z = -0.07;} break;
        case 10: if(this->pose == 0){z = -0.11;}else{z = -0.07;} break;
        default: z = -0.11;
    }
    return z;
}

