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
    
	// Setup the robot
	RobotInterface *robot = new RobotInterface(argc[1],0);
	RobotPose robotPose(robot, argc[2]);
	robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);

	
	/*
	 * Move forward down the hallway
	 */
	
	/*
	//Robot 2
	robotPose.moveToCell(5,2);
	robotPose.moveToCell(4,2);
	robotPose.moveToCell(4,1);
	*/
	
	
	//Robot 1
	robotPose.moveToCell(0,3);
	robotPose.moveToCell(0,4);
	robotPose.moveToCell(1,4);
	
	
	//robotPose.centerInCell();

	delete(robot);
	return 0;


}

