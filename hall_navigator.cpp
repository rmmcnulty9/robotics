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
	  //float goal_theta = acos((x-pose_kalman.x)/sqrt((x-pose_kalman.x)*(x-pose_kalman.x)+(y-pose_kalman.y)*(y-pose_kalman.y)));
	  float goal_theta = M_PI_2;
	int i=1;
	for (;i<=2;i++){
	  robotPose.moveTo(0.0,i* 65.0, goal_theta);
	  printf("\n\n CELL %d \n\n",i);
	}
	i-=1;
	goal_theta = 0.0;
	for(int j=1;j<=3;j++){
	  robotPose.moveTo(j*65.0,i*65.0, goal_theta);
	  printf("\n\nCell %d %d\n\n",i,j);
	  if(j==1){
	  robotPose.changeUncertainty(uncertainty_weak_ns);
	    
	  }
	  
	}
	
	delete(robot);
	return 0;


}

