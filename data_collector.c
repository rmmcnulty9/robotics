#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
int main(int argv, char **argc) {
        robot_if_t ri;

        // Make sure we have a valid command line argument
        if(argv <= 1) {
                printf("Usage: robot_test <address of robot>\n");
                exit(-1);
        }

        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");

        // Action loop
        do {
                // Update the robot's sensor information
                if(ri_update(&ri) != RI_RESP_SUCCESS) {
                        printf("Failed to update sensor information!\n");
                        break;
                }
                struct timeval now;
                gettimeofday(&now,NULL);
                printf("N %ld.%ld %d %d %f\n",now.tv_sec, now.tv_usec, ri_getX(&ri), ri_getY(&ri), ri_getTheta(&ri));
                
                printf("E %ld.%ld %d %d %d\n", now.tv_sec, now.tv_usec, ri_getWheelEncoder(&ri,RI_WHEEL_LEFT),
                                      ri_getWheelEncoder(&ri,RI_WHEEL_RIGHT),
                                      ri_getWheelEncoder(&ri,RI_WHEEL_REAR));
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)){
                  ri_move(&ri, RI_MOVE_FORWARD, RI_FASTEST);
                }else{
                  break;
                }
        } while(1);

        return 0;
}
