#ifndef COMMON_H
#define COMMON_H

#define NUM_CLASSES 11            //lego block classes
#define OUTPUT_FILE "OUTPUT.txt"  //where to print the output
#define LOG_FILE "log.txt"        //log file

#include <iostream>
#include <vector>

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


/**
 * @brief Get lego name of class [lego_int]
 * 
 * @param lego_int class integer identifier
 * @return std::string lego class name
 */
std::string get_lego_string(int lego_int);

#endif
