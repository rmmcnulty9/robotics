/*
 * Some shared Constants/Structs across this Rob
 * */
 #ifndef __CONSTANTS__
 #define __CONSTANTS__
typedef struct {
    float x;
    float y;
    float theta;
} pose;


const float we_to_cm = 1/2.0;
const float ns_to_cm = 1/300.0;
const float robot_radius = 4.0;
#endif
