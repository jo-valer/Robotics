#include "my_world/World.h"

using namespace std;

World::World(){
    //Init missing area (number of target area on the map)
    missingArea = CARD_TARGET;

    //=====INIT MAP=====
    ifstream in(WORLD_MAP_FILE);    //open input file
    int cardBasket = CARD_BASKET;   //number of baskets (costant)
    int cardTarget = CARD_TARGET;   //number of target zones (constant)

    N=cardBasket+cardTarget+1;

    graph.resize(N);
    mapArea.resize(CARD_BASKET+CARD_TARGET+1);

    double input_x, input_y;
    TargetArea new_ta;
    Coordinate pos_ta;

    //Read initial position coordinates
    in>>input_x>>input_y;
    startPosition.setX(input_x);
    startPosition.setY(input_y);
    mapArea[0].setX(input_x);
    mapArea[0].setY(input_y);

    //to allign targetArea identifiers we have to insert something in targetArea[0]
    new_ta.setPosition(startPosition);
    targetArea.push_back(new_ta);

    //Read target area coordinates
    for(int i=1; i<=cardTarget; i++){
        in>>input_x>>input_y;
        mapArea[i].setX(input_x);
        mapArea[i].setY(input_y);
        
        pos_ta.setX(input_x);
        pos_ta.setY(input_y);
        new_ta.setPosition(pos_ta);

        targetArea.push_back(new_ta);
    }

    //Read basket coordinates
    for(int i=cardTarget+1; i<=(cardBasket+cardTarget); i++){
        in>>input_x>>input_y;
        mapArea[i].setX(input_x);
        mapArea[i].setY(input_y);
    }
    
    //Read number of routes
    in>>M;

    //Read undirected graph from file
    //w = weight ; in our case, distance
    int from, to, w;
    double d;
    Coordinate areaFrom, areaTo;
    for (int j=0; j<M; j++) {
        in >> from >> to;

        //Get coordinates of area from and to
        areaFrom=mapArea[from];
        areaTo=mapArea[to];

        //Compute distance between two points
        d = two_points_distance(areaFrom, areaTo);
        
        //Convert d into an integer distance
        w = d*100;

        graph[from].vic.push_back(make_pair(to, w));
        graph[to].vic.push_back(make_pair(from, w));
    }
    //===end of map initialization===

    //Init visited vector
    for(int i=0; i<=CARD_TARGET; i++){
        visited[i]=false;
    }
}

void World::print_map(){
    for(int i=0; i<N; i++){
        cout<<"ZONA["<<i<<"]"<<endl;
        for (pair<int, int> adj : graph[i].vic) {
            int v = adj.first;   //adjacent node
            int w = adj.second;  //weight

            cout<<"adj: "<<v<<" - distance="<<w<<endl;
        }
    }
}

bool World::complete(){
    return (missingArea==0);
}

void World::areaCompleted(){
    if(missingArea>0){
        missingArea = missingArea-1;
    }
}