//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>
#include "CameraPose.cpp"
#include "shared_constants.h"
#include "RobotPose.cpp"
#include "PIDController.h"
extern "C" {
#include "Kalman/kalmanFilterDef.h"
}

int main(int argv, char **argc)
{
	// Make sure we have a valid command line argument
	if(argv <= 2) {
		std::cout << "Usage: robot_test <address of robot> " << std::endl;
		exit(-1);
	}
    
	if(0==strncmp(argc[1],"rosie",strlen("rosie"))){
	      ns_x_to_cm = rosie_ns_x_to_cm;
	      ns_y_to_cm = rosie_ns_x_to_cm;
	      we_to_cm = rosie_we_to_cm;
	      ns_theta_offsets = rosie_ns_theta_offsets;
	      
	      // If rosies's WE are bad set them to NS???
	      
	}else if(0==strncmp(argc[1],"bender",strlen("bender"))){
	      ns_x_to_cm = bender_ns_x_to_cm;
	      ns_y_to_cm = bender_ns_x_to_cm;
	      we_to_cm = bender_we_to_cm;
	      ns_theta_offsets = bender_ns_theta_offsets;
	}else{
	      printf("Bot Not supported!\n"); 
	}
    
	// Setup the robot
	RobotInterface *robot = new RobotInterface(argc[1],0);
	RobotPose robotPose(robot, argc[2]);
	robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);
	robotPose.getMap();
	robotPose.printMap();


	/*
	while(true){
	robotPose.updatePosition(false);
	robotPose.printPoses();
	}*/
	/*
	 * Move forward down the hallway
	 */
	
	/*
	//Robot 2
	robotPose.moveToCell(5,2);
	robotPose.moveToCell(4,2);
	robotPose.moveToCell(4,1);
	robotPose.moveToCell(4,0);
	robotPose.moveToCell(3,0);
	robotPose.moveToCell(2,0);
	robotPose.moveToCell(2,1);	
	*/
	/*
	robotPose.moveToCell(5,2);
	robotPose.moveToCell(4,2);
	robotPose.moveToCell(4,1);
	robotPose.moveToCell(4,0);
	robotPose.moveToCell(5,0);
	robotPose.moveToCell(6,0);
	*/
	//Robot 1
	/*
	robotPose.moveToCell(0,1);
	robotPose.moveToCell(0,0);
	robotPose.moveToCell(1,0);
	robotPose.moveToCell(2,0);
	robotPose.moveToCell(3,0);
	robotPose.moveToCell(3,1);
	*/
	/*
	robotPose.moveToCell(1,2);
	robotPose.moveToCell(2,2);
	robotPose.moveToCell(2,1);
	robotPose.moveToCell(2,0);
	robotPose.moveToCell(3,0);
	*/
	
	robotPose.moveToCell(4,4);
	robotPose.moveToCell(4,3);
	robotPose.moveToCell(4,2);
	robotPose.moveToCell(4,1);
	robotPose.moveToCell(4,0);
	robotPose.moveToCell(3,0);
	//robotPose.centerInCell();

	delete(robot);
	return 0;


}

