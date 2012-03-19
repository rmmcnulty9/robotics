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
	if(argv <= 1) {
		std::cout << "Usage: robot_test <address of robot> " << std::endl;
		exit(-1);
	}
    
	// Setup the robot
	RobotInterface *robot = new RobotInterface(argc[1],0);
	RobotPose robotPose(robot);
	robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);


	/*
	 * Move forward down the hallway
	 */
	int ctr = 0;
	while(!robot->IR_Detected() && ctr<=10){
		printf("CELL %d ===============================================\n", ctr);
		robotPose.moveToCell(RobotPose::FORWARD);
		ctr++;
	}
	printf("CELL TURN %d ===============================================\n", ctr);
	robotPose.moveToCell(RobotPose::RIGHT);
        ctr++;
	
	while(!robot->IR_Detected() && ctr<=20){
		printf("CELL %d ===============================================\n", ctr);
		robotPose.moveToCell(RobotPose::FORWARD);
		ctr++;
	}

	delete(robot);
	return 0;


}

