//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>
#include "shared_constants.h"
#include "RobotPose.cpp"


int main(int argv, char **argc)
{
	// Make sure we have a valid command line argument
	if(argv <= 2) {
		std::cout << "Usage: robot_test <address of robot> <fir_coef_files>" << std::endl;
		exit(-1);
	}
    
	RobotInterface *robot = new RobotInterface(argc[1],0);
	char *coefFile = argc[2];
	RobotPose robotPose(robot, coefFile);
	pose we;
	bool turn = false;
	do {
		  // Update the robot's sensor information
		  robotPose.updatePosition();
		  robotPose.getPositionWE(we);
		  std::cout << we.x << ",\t" << we.y << ",\t" << we.theta << std::endl;
		  if(we.y > 300 && !turn){
			robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST); 
			robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST); 
			robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST); 
			robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST); 
			turn = true;
		  }
                // Move forward unless there's something in front of the robot
		  if(!robot->IR_Detected())
			  robot->Move(RI_MOVE_FORWARD, RI_FASTEST);
	
	} while(1);
	delete(robot);
	return 0;
}

