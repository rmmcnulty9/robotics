//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>
#include "shared_constants.h"
#include "RobotPose.cpp"
extern "C" {
#include "Kalman/kalmanFilterDef.h"
}

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
	
  
		robotPose.updatePosition();
   // printf("Moving forward\n");
	for(int i=0; i<60; i++){
		  // Update the robot's sensor information
		  robotPose.updatePosition();

		  // Move forward unless there's something in front of the robot

		   if(!robot->IR_Detected()){
			  robot->Move(RI_MOVE_FORWARD, RI_FASTEST);
		  }
		  else{
			//std::cout << "Wall!\n";
        break;
		  }	 
	}
  
	//std::cout << "Turning\n";
	for(int i=0; i<4; i++){
		robotPose.updatePosition(true);
		robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST);
	}


	//std::cout << "Moving\n";
	for(int i=0; i<60; i++){
		  // Update the robot's sensor information
		  robotPose.updatePosition();
		  // Move forward unless there's something in front of the robot
		 if(!robot->IR_Detected()){
			  //robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST);
			  robot->Move(RI_MOVE_FORWARD, RI_FASTEST);
		  }
		  else{
			break;
		  }	 
	}

	delete(robot);
	return 0;
}

