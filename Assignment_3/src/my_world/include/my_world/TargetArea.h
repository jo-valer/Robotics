#ifndef TARGET_AREA_H
#define TARGET_AREA_H

#include <my_world/Lego.h>
#include <my_world/Coordinate.h>
#include <vector>

#define MAX_AREA_LEGO 3

using namespace std;

class TargetArea{
    private:
        Coordinate pos;
        int numLego;
        vector<Lego> legoList;  //assumption: there can be maximum MAX_AREA_LEGO in the area

    public:
        TargetArea();
        
        //Set target area coordinates
        void setPosition(Coordinate _p);

        //Remove a lego from the target area
        void removeLego();
        
        //Add lego l to the target area
        void addLego(Lego l);

        //How many lego in the area (-1 if the area has not been explored yet)
        int getNumLego();

        //Return true iff there is no lego in the area
        bool isEmpty();

        //Return lego legoList[index]
        //Print error message if index is not between 0 and MAX_AREA_LEGO-1
        Lego getLego(int index);
        
        //Return the next lego which has to be picked up from the area
        Lego getNextLego();        
};

#endif

