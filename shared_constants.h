/*
 * Some shared Constants/Structs across this Rob
 * */
#include <math.h>

 #ifndef __CONSTANTS__
 #define __CONSTANTS__
typedef struct {
    float x;
    float y;
    float theta;
} pose;

const float we_to_rad = M_PI/-120;
const float we_to_cm = 2.0; //1.0/2.0
const float ns_x_to_cm = 1.0/30.0; //1.0/45
const float ns_y_to_cm = 1.0/40.0;
const float robot_diameter_cm = 29.0;
//const float robot_diameter = 16.0;
#endif
