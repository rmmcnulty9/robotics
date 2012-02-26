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
	
	//Base 0 to Base 1
	robotPose.moveTo(0.0,350.0); // y =  134.5 inches x=0

	//Base 1 to Base 2 -185
	robotPose.moveTo(-180.0,259.0); // x=-73 y=134.5-32.5 = 102

	robotPose.resetWEPose();
	robotPose.moveTo(-180.0,340.0); // checkpoint between base 2 & 3
	//Base 2 to Base 3
	robotPose.moveTo(-354.0,424.0); // x=-73-66.5 = -139.5  y=102+65 = 167

	robotPose.resetWEPose();
	robotPose.changeWEScalingConstant(1.9);
	
	robotPose.moveTo(-443.0,260.0); // checkpoint between base 3 & 4
	//Base 3 to Base 4
	robotPose.moveTo(-443.0,76.0); // x=-139.5-35 = -174.5  y=167-137=30
	//Base 4 to Base 0
	robotPose.moveTo(0.0,0.0); // x=0  y=0
	delete(robot);
	return 0;


}

