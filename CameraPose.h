
#ifndef __CAMERA_POSE__
#define __CAMERA_POSE__

#include "shared_constants.h"
#include "robot_if++.h"
#include "robot_color.h"
#include <list>

using namespace std;
/*
 * Holds pair of matching squares
 */
typedef struct {
	squares_t* left;
	squares_t* right;
} squarePair;


class CameraPose {
  
	public:
	CameraPose(RobotInterface *r);
	~CameraPose();
  
	void updateCamera();
	void displayImages();
	list<squarePair> matchSquares(squares_t *squares);
	void drawSquares(squares_t *squares, CvScalar displayColor);
	void strafeTo(float delta_y);
	void printCenters(list<squarePair> pairs);
	int getCenterError(list<squarePair> pairs);
	private:
  
	RobotInterface *robot;
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *filteredImage;
	static const int MIN_SQUARE = 200;
  
};


#endif
