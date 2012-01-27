/*
 * Some shared Constants/Structs across this Rob
 * */
#ifndef __SHARED_CONSTANTS__
#define __SHARED_CONSTANTS__
 
typedef struct {
    float x;
    float y;
    float theta;
} pose;


const float we_to_cm = 1/20.0;
const float ns_to_cm = 1/300.0;

#endif 
