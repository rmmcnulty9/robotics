//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>
#include "shared_constants.h";

int main(int argc, char **argv)
{
	// Make sure we have a valid command line argument
	if(argv <= 2) {
		std::cout << "Usage: robot_test <address of robot> <fir_coef_files>" << std::endl;
		exit(-1);
	}
    
	RobotInterface *r = new RobotInterface(argc[1],0);
	RobotPose robot(r);
	pose *we;
	do {
		  // Update the robot's sensor information
		  robot.updatePosition();
		  robot.getPositionWE(we);
		  std::cout << we->x << "," << we->y << "," << we->theta << std::endl;
                // Move forward unless there's something in front of the robot
		  if(!r->IR_Detected())
			  robot->Move(RI_MOVE_FORWARD, RI_FASTEST);
	
	} while(1);
	delete(r);
	return 0;
}

