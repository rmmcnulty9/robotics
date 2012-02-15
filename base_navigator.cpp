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
	//char *coefFile = argc[2];
	RobotPose robotPose(robot);
	pose we;
	
	//robotPose.turnTo(1.57);
//	robotPose.moveTo(100,0);
/*
int i=0;
for(;i<40;i++){
robot->update();
printf("%f\n", robot->Theta()*(180/M_PI));
}
*/

//	robotPose.moveTo(-100, -1);
	robotPose.moveTo(0,-100);
	
//robotPose.moveTo(-100,100);	//45 deg
//robotPose.moveTo(-100,-100);	//135 deg
//robotPose.moveTo(100,100);	//-45 deg
//robotPose.moveTo(100,-100);	//-135 deg
	
	return 0;
	
	robotPose.updatePosition();
   // printf("Moving forward\n");
/*	for(int i=0; i<60; i++){
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
  */
	//std::cout << "Turning\n";
	robot->reset_state();
	
	for(int i=0; i<4; i++){
		robotPose.updatePosition(true);
		robot->Move(RI_TURN_LEFT_20DEG, RI_FASTEST);
	}
	printf("%d\n",robot->getWheelEncoderTotals(RI_WHEEL_REAR));
return 0;

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

