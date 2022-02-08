#include <iostream>
#include "my_world/TargetArea.h"

using namespace std;

TargetArea::TargetArea(){
    //Init lego number
    this->numLego = -1;

    //Init target area coordinates
    (this->pos).setX(0.0);
    (this->pos).setY(0.0);

    //Init lego vector
    Lego l("unknown", -1.0, -1.0, 11, 0.0, 0);
    for(int i=0; i<MAX_AREA_LEGO; i++){
        legoList.push_back(l);
    }
}


void TargetArea::setPosition(Coordinate _p){
    (this->pos).setX(_p.getX());
    (this->pos).setY(_p.getY());
}


void TargetArea::removeLego(){
    if(numLego>0){
        numLego = numLego-1;
    }else{
        cout<<"Warning: there are no lego in this area"<<endl;
    }
}


void TargetArea::addLego(Lego l){

    if(numLego==-1){    //the area has been just descoverd
        numLego=0;
    }

    if(numLego<MAX_AREA_LEGO){
        legoList[numLego].setName(l.getName());
        legoList[numLego].setX(l.getX());
        legoList[numLego].setY(l.getY());
        legoList[numLego].setClasse(l.getClasse());

        numLego = numLego+1;
    }else{
        cout<<"Warning: this area is full"<<endl;
    }
}

int TargetArea::getNumLego(){
    return numLego;
}

bool TargetArea::isEmpty(){
    return (numLego==0);
}

Lego TargetArea::getLego(int index){
    Lego emptyLego;

    if(index>=0 && index<numLego){
        return legoList[index];
    }else{

        if(isEmpty()){
            cout<<"Warning: it is not possible to find a lego in this area"<<endl;    
        }else{
            cout<<"Warning: invalid index. Range of valid indexes ["<<0<<" - "<<numLego-1<<"]"<<endl;
        }
        return emptyLego;
    }
}

Lego TargetArea::getNextLego(){
    Lego emptyLego;
    
    if(!( this->isEmpty() )){
        return legoList[numLego-1];
    }

    cout<<"Warning: it is not possible to find a lego in this area"<<endl;
    return emptyLego;
}
