#include "my_world/common.h"

using namespace std;


string get_lego_string(int lego_int){
    string lego_string;
    switch (lego_int){
        case 0: lego_string = "X1_Y1_Z2"; break;
        case 1: lego_string = "X1_Y2_Z1"; break;
        case 2: lego_string = "X1_Y2_Z2"; break;
        case 3: lego_string = "X1_Y2_Z2_CHAMFER"; break;
        case 4: lego_string = "X1_Y2_Z2_TWINFILLET"; break;
        case 5: lego_string = "X1_Y3_Z2"; break;
        case 6: lego_string = "X1_Y3_Z2_FILLET"; break;
        case 7: lego_string = "X1_Y4_Z1"; break;
        case 8: lego_string = "X1_Y4_Z2"; break;
        case 9: lego_string = "X2_Y2_Z2"; break;
        case 10: lego_string = "X2_Y2_Z2_FILLET"; break;
        default: lego_string = "UNKNOWN";
    }
    return lego_string;
}

std::pair<Coordinate, Coordinate> tangent_point(double cx, double cy, double px, double py){
    double a = radius_targetArea;
    double b;
    double d;
    double d1;
    double d2;
    double th;

    double t1y;
    double t1x;

    double t2x;
    double t2y;

    Coordinate t1;
    Coordinate t2;

    b=sqrt(pow((px - cx),2) + pow((py - cy),2));    //distance CP
    th=acos(a/b);                                   //angle theta
    d = atan2(py-cy, px-cx);                        //direction angle of point P from C
    d1 = d + th;                                    //direction angle of point T1 from C
    d2 = d - th;                                    //direction angle of point T2 from C

    //point one coordinate
    t1x = cx + a * cos(d1);
    t1y = cy + a * sin(d1);

    //point two coordinate
    t2x = cx + a * cos(d2);
    t2y = cy + a * sin(d2);

    //output
    t1.setX(t1x);
    t1.setY(t1y);

    t2.setX(t2x);
    t2.setY(t2y);
    //...

    return std::make_pair(t1, t2);
}

double two_points_distance(Coordinate p1, Coordinate p2){
    double p1_x = p1.getX();
    double p1_y = p1.getY();

    double p2_x = p2.getX();
    double p2_y = p2.getY();

    return sqrt(pow(p2_x-p1_x,2)+pow(p2_y-p1_y, 2));
}

/*
void init_map(vector<nodo>& graph, vector<Coordinate>& mapArea, Coordinate& startPosition){
    ifstream in(WORLD_MAP_FILE);    //open input file
    int cardBasket = CARD_BASKET;   //number of baskets (costant)
    int cardTarget = CARD_TARGET;   //number of target zones (constant)

    //targetArea.resize(cardTarget);
    //basketArea.resize(cardBasket);

    double input_x, input_y;

    //Read initial position coordinates
    in>>input_x>>input_y;
    startPosition.setX(input_x);
    startPosition.setY(input_y);
    mapArea[0].setX(input_x);
    mapArea[0].setY(input_y);
*/
    /*
    commentato
    //Read target area coordinates
    for(int i=0; i<cardTarget; i++){
        in>>input_x>>intput_y;
        tmp.setX(input_x);
        tmp.setY(input_y);
        targetArea[i]=tmp;
    }

    //Read basket area coordinates
    for(int i=0; i<cardBasket; i++){
        in>>input_x>>intput_y;
        tmp.setX(input_x);
        tmp.setY(input_y);
        basketArea[i]=tmp;
    }
    commentato
    */
/*

    //Read all map area (basket + lego) coordinates
    for(int i=1; i<(cardBasket+cardTarget+1); i++){
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
}
*/
void dijkstra(vector<nodo>& graph, int S, vector<int>& distance, vector<int>& parent){
    priority_queue<vector<int>, vector<vector <int>>, cmp> pq;   //priority queue

    distance[S] = 0;
    parent[S]=-1;

    vector<int> insertInto_pq = {S, 0, S};  //inizializzazione: da sorgente a sorgente costo 0 
    pq.push(insertInto_pq);

    while (!pq.empty()) {
        vector<int> prossimoNodo = pq.top(); //{current node, cost, parent}
        pq.pop();

        int u = prossimoNodo[0];    //current node

        for (pair<int, int> adj : graph[u].vic) {

            int v = adj.first;   //adjacent node
            int w = adj.second;  //weight

            int alt = distance[u] + w;

            if(alt<distance[v]){
                insertInto_pq = {v, alt, u};
                pq.push(insertInto_pq);
                distance[v] = alt;
                parent[v] = u;
            }
        }
    }
    /**---*/

    /*
    for(int i=0; i<N; i++){
        cout << i << " - " << distance[i] << "\n";
    }
    */
}

void printPath(int s, int r, vector<int>& parent, vector<int>& path){
    if(s==r){
        path.push_back(r);
    }else{
        printPath(s, parent[r], parent, path);
        path.push_back(r);
    }
}

/*
void print_map(const vector<nodo>& graph){
    for(int i=0; i<N; i++){
        cout<<"ZONA["<<i<<"]"<<endl;
        for (pair<int, int> adj : graph[i].vic) {
            int v = adj.first;   //adjacent node
            int w = adj.second;  //weight

            cout<<"adj: "<<v<<" - distance="<<w<<endl;
        }
    }
}
*/