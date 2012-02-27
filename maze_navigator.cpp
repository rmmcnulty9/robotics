//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include "robot_if++.h"
#include "robot_color.h"
#include <iostream>
#include <string>
#include "shared_constants.h"
#include "RobotPose.cpp"
#include "CameraPose.cpp"
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
	CameraPose cameraPose(robot);
	while(true){
		cameraPose.updateCamera();
	}
        

	delete(robot);
	return 0;


}

