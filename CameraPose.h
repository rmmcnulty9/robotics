
#ifndef __CAMERA_POSE__
#define __CAMERA_POSE__

#include "shared_constants.h"
#include "robot_if++.h"
#include "robot_color.h"

/*
 * Holds pair of matching squares
 */
typedef struct {
	squares_t left;
	squares_t right;
} squarePair;


class CameraPose {
  
	public:
	CameraPose(RobotInterface *r);
	~CameraPose();
  
	void updateCamera();
	void displayImages();
	squarePair* matchSquares(squares_t *squares);
	void drawSquares(squares_t *squares, CvScalar displayColor);
	private:
  
	RobotInterface *robot;
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *filteredImage;
	static const int MIN_SQUARE = 200;
  
};


#endif
