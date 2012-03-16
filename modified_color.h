/* Colors used by OpenCV */

#ifndef __RC_COLORS_H__
#define __RC_COLORS_H__

#define RC_LOW(x)	cvScalar(x - 5, 100, 100, 0)
#define RC_HIGH(x) 	cvScalar(x + 5, 255, 255, 0)

/* Pink */
#define RC_PINK1	0
#define RC_PINK1_LOW	cvScalar(0, 100, 100, 0)
#define RC_PINK1_HIGH	cvScalar(20/2, 255, 255, 0)

#define RC_PINK2	359
#define RC_PINK2_LOW	cvScalar(340/2, 100, 100, 0)
#define RC_PINK2_HIGH	cvScalar(359/2, 255, 255, 0)

/* Yellow */
#define RC_YELLOW	30
#define RC_YELLOW_LOW	RC_LOW(RC_YELLOW)
#define RC_YELLOW_HIGH	RC_HIGH(RC_YELLOW)

/* Blue */
#define RC_BLUE		100
#define RC_BLUE_LOW	RC_LOW(RC_BLUE)
#define RC_BLUE_HIGH	RC_HIGH(RC_BLUE)

/* Green */
#define RC_GREEN	50
#define RC_GREEN_LOW	RC_LOW(RC_GREEN)
#define RC_GREEN_HIGH	RC_HIGH(RC_GREEN)

/* Purple */		
#define RC_PURPLE	140
#define RC_PURPLE_LOW	RC_LOW(RC_PURPLE)
#define RC_PURPLE_HIGH	RC_HIGH(RC_PURPLE)

#endif /* __RC_COLORS_H__ */
