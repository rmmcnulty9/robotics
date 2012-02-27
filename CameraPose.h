
#ifndef __CAMERA_POSE__
#define __CAMERA_POSE__

#include "shared_constants.h"
#include "robot_if++.h"
#include "robot_color.h"

class CameraPose {
  
	public:
	CameraPose(RobotInterface *r);
	~CameraPose();
  
	void updateCamera();
	void displayImage();
	void findSquares();
	private:
  
	RobotInterface *robot;
	IplImage *image;
	IplImage *hsv;
	IplImage *filtered;

  
};


#endif
