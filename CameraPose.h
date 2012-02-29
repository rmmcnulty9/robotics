
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
	void displayImages();
	void matchSquares(squares_t *squares);
	void drawSquares(squares_t *squares, CvScalar displayColor);
	private:
  
	RobotInterface *robot;
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *filteredImage;
	static const int MIN_SQUARE = 200;
  
};


#endif
