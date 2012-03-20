//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>
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
    
	RobotInterface *robot = new RobotInterface(argc[1],0);
	
	RobotPose robotPose(robot);
	
	for (int i=1;i<=5;i++){
	  robotPose.moveTo(0.0,i* 65.0);
	  printf("\n\n CELL %d \n\n",i);
	}
	delete(robot);
	return 0;


}

