/*
 * Hall Navigator Class
 *  Used to move the robot down the hallway with the aid of RobotPose and CameraPose
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

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
	//float goal_theta = acos((x-pose_kalman.x)/sqrt((x-pose_kalman.x)*(x-pose_kalman.x)+(y-pose_kalman.y)*(y-pose_kalman.y)));
	float goal_theta = M_PI_2;
	
	//Move the robot 5 cells forward and 3 cells to the right
	int i=1, j=1;
	for (;i<=5;i++){
		robotPose.moveTo(0.0,i* 65.0, goal_theta);
		printf("\n\n CELL %d %d\n\n",i,j);
		//Adjust for NS skew
		//if(i==4){robotPose.changeUncertainty(uncertainty_weak_ns);}
		if(i==5){
			robotPose.changeWEScalingConstant(20.0); 
	  	}
	}
	//Adjust for WE drift
	robotPose.changeWEScalingConstant(9.0); 
	i-=1;
	//Change direction
	goal_theta = 0.0;
	
	for(;j<=4;j++){
		robotPose.moveTo(j*65.0,i*65.0, goal_theta);
		printf("\n\n CELL %d %d\n\n",i,j);
		//Adjust for poor NS readings
		if(j==1){
			robotPose.changeUncertainty(uncertainty_weak_ns); 
		}
		else if(j==3){
			robotPose.changeUncertainty(uncertainty_weakest_ns); 
		}
	}
	
	delete(robot);
	return 0;
}

