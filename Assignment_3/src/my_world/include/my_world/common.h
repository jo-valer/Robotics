#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <queue>
#include "my_world/Coordinate.h"

#define NUM_CLASSES 11                                               //lego block classes
#define OUTPUT_FILE "OUTPUT.txt"                                     //where to print the output
#define LOG_FILE "log.txt"                                           //log file
#define DYNAMIC_LINK_PATH "src/my_world/world/dynamic_links.txt"     //dynamic link path

//Lego class names
enum Brick { X1_Y1_Z2 = 0,
             X1_Y2_Z1 = 1,
             X1_Y2_Z2 = 2,
             X1_Y2_Z2_CHAMFER = 3,
             X1_Y2_Z2_TWINFILLET = 4,
             X1_Y3_Z2 = 5,
             X1_Y3_Z2_FILLET = 6,
             X1_Y4_Z1 = 7,
             X1_Y4_Z2 = 8,
             X2_Y2_Z2 = 9,
             X2_Y2_Z2_FILLET = 10,
             UNKNOWN = 11};

#define WORLD_MAP_FILE "src/my_world/src/map.txt"
#define CARD_BASKET 11
#define CARD_TARGET 4

using namespace std;

//===WORLD CONSTANTS===
const double radius_targetArea=0.9;
//=====================


/**
 * @brief Get lego name of class [lego_int]
 * 
 * @param lego_int class integer identifier
 * @return std::string lego class name
 */
std::string get_lego_string(int lego_int);

/**
 * @brief 
 * Gets the coordinates of the two tangent points on the target area circumference with center in (cx, cy). Tanget lines from P(px, py).
 * @param cx circumference center x
 * @param cy circumference center y
 * @param px point x
 * @param py point y
 */
std::pair<Coordinate, Coordinate> tangent_point(double cx, double cy, double px, double py);

/**
 * @brief 
 * Compute the distance between two points in 2D 
 */
double two_points_distance(Coordinate p1, Coordinate p2);

//===DATA STRUCTURES FOR DIJKSTRA ALGORITHM===
struct nodo {
    std::vector<std::pair<int, int>> vic; //adjacent node - weight
    //vector< vector<int> > vic; //alternativa con vector invece che pair
    bool visited;
};

/*cmp structure for priority queue*/
struct cmp{
    bool operator()(std::vector<int> a, std::vector<int> b){
        return a[1] > b[1];
    }
};
//===========================================

//==DIJKSTRA ALGORITHM===

/**
 * @brief 
 * Return tree of minimum distances. S is the source node, root of the tree.
 */
void dijkstra(vector<nodo>& graph, int S, std::vector<int>& distance, std::vector<int>& parent);

/**
 * @brief 
 * The minimum path tree is stored in a parent vector.
 * The function writes into vector path the min distance path
 * from node s to node r
 * @param s start node 
 * @param r destination
 * @param parent parent vector
 */
void printPath(int s, int r, std::vector<int>& parent, std::vector<int>& path);
//=======================

/**
 * @brief 
 * Read world map from file.
 * Assumptions:
 *  - The structure of the file il well formed and suit the protocol
 *  - There are always 4 target areas
 *  - There are always 11 baskets
 * 
 * The procedure reads all the locations and for each route computes the lenght of the route.
 */
//void init_map(vector<nodo>& graph, vector<Coordinate>& mapArea, Coordinate& startPosition);

/**
 * @brief 
 * Print world map (the graph)
 */
void print_map(const vector<nodo>& graph);


#endif
