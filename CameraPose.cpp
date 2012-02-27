#include <robot_if++.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cstdio>
#include "CameraPose.h"
#include "shared_constants.h"
#include "robot_if++.h"
#include "robot_color.h"


CameraPose::CameraPose(RobotInterface *r){
	robot = r;
	currentImage = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
	cvNamedWindow("Rovio Camera", CV_WINDOW_AUTOSIZE);
	// Setup the camera
        if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_320, RI_CAMERA_QUALITY_HIGH)) {
                std::cout << "Failed to configure the camera!" << std::endl;
                exit(-1);
        }
}
CameraPose::~CameraPose(){}

void CameraPose::updateCamera(){
	// Update the robot's sensor information
	robot->update();
	// Get the current camera image
	robot->getImage(currentImage);
	displayImage();
}

void CameraPose::displayImage(){
	cvShowImage("Rovio Camera", currentImage);
	cvWaitKey(10);
}