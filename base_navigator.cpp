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
	//Base 0 to Base 1
	robotPose.moveTo(0,341); // y =  134.5 inches x=0
	//Base 1 to Base 2
	robotPose.moveTo(-185,259); // x=-73 y=134.5-32.5 = 102
	//Base 2 to Base 3
	robotPose.moveTo(-354,424); // x=-73-66.5 = -139.5  y=102+65 = 167
	//Base 3 to Base 4
	robotPose.moveTo(-443,76); // x=-139.5-35 = -174.5  y=167-137=30
	//Base 4 to Base 0
	robotPose.moveTo(0,0); // x=0  y=0


	//robotPose.turnTo(1.57);
//	robotPose.moveTo(100,0);

//	robotPose.moveTo(-100, -1);
//	robotPose.moveTo(0,-100);
	
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

