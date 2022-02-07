#ifndef WORLD_H
#define WORLD_H

#include "my_world/common.h"
#include "my_world/TargetArea.h"

//#define WORLD_MAP_FILE "src/my_world/src/map.txt"
//#define CARD_BASKET 5
//#define CARD_TARGET 4

//#define RADIUS_TARGET_AREA 0.9

using namespace std;

class World{
    private:
        int missingArea;                    //number of area to be completed (init: CARD_TARGET)
        
    public:
        vector<nodo> graph;                 //undirected graph
        int N;                              //number of nodes (constant)
        int M;                              //number of edges (=routes)

        Coordinate startPosition;           //where the mir robot is spawn
        vector<TargetArea> targetArea;      //target zones coordinates : where the legos are
        vector<Coordinate> mapArea;         //basket or lego area
        bool visited[CARD_TARGET];          //visited[i]=true iff object localization and classification has been completed on area number i

        World();

        //TRUE iff the exuction has been completed: we have done everything in all the target areas
        bool complete();
        
        //Print world graph
        void print_map();
        
        //Decrese the number of missing areas
        void areaCompleted();
};

#endif

