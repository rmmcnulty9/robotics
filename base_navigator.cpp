//The start of a program that uses 

#include <stdio.h>
#include <robot_if++.h>
#include <iostream>
#include <string>


int main(int argc, char **argv)
{
	// Make sure we have a valid command line argument
    if(argv <= 1) {
        std::cout << "Usage: robot_test <address of robot> <fir_coef_files>" << std::endl;
        exit(-1);
    }
    RobotPose robot()

	return 0;
}

