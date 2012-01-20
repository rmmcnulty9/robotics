#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>

int main(int argc, char **argv) {
        robot_if_t ri;

        // Make sure we have a valid command line argument
        if(argc != 2) {
                printf("Usage: robot_test <address of robot>\n");
                exit(-1);
        }

        // Setup the robot with the address passed in
        if(ri_setup(&ri, argv[1], 0))
                printf("Failed to setup the robot!\n");
        
        
        //Variables to dump the robot data out
        int d_left=0, d_right=0, d_rear=0.0;
        int t_left=0, t_right=0, t_rear=0.0;
          
          int x=0, y=0;
          float theta = 0.0;
        
        // Action loop
        do {
                // Update the robot's sensor information
                if(ri_update(&ri) != RI_RESP_SUCCESS) {
                        printf("Failed to update sensor information!\n");
                        break;
                }
               // printf("N %ld.%ld %d %d %f\n",now.tv_sec, now.tv_usec, ri_getX(&ri), ri_getY(&ri), ri_getTheta(&ri));
                d_left = ri_getWheelEncoder(&ri,RI_WHEEL_LEFT);
                d_right = ri_getWheelEncoder(&ri,RI_WHEEL_RIGHT);
                d_rear = ri_getWheelEncoder(&ri,RI_WHEEL_REAR);
                t_left = ri_getWheelEncoderTotals(&ri,RI_WHEEL_LEFT);
                t_right = ri_getWheelEncoderTotals(&ri,RI_WHEEL_RIGHT);
                t_rear = ri_getWheelEncoderTotals(&ri,RI_WHEEL_REAR);
               
               x = ri_getX(&ri);
               y = ri_getY(&ri);
               theta = ri_getTheta(&ri);
               
               printf("%d %d %f %d %d %d %d %d %d\n", x, y, theta, d_left, d_right, d_rear, t_left, t_right, t_rear);
                  
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)){
                  ri_move(&ri, RI_MOVE_FORWARD, RI_FASTEST);
                }else{
                  break;
                }
        } while(1);

        return 0;
}

