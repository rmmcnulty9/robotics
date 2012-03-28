/*
 * Modified color header
 *  Changes made to standard colors to aid in filtering
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

#ifndef __RC_COLORS_H__
#define __RC_COLORS_H__

#define RC_LOW(x)	cvScalar(x - 5, 100, 100, 0)
#define RC_HIGH(x) 	cvScalar(x + 5, 255, 255, 0)

/* Pink modified to capture both low and high end hues*/
#define RC_PINK1	0
#define RC_PINK1_LOW	cvScalar(0.0, 90.0, 90.0, 0.0)
#define RC_PINK1_HIGH	cvScalar(25.0 / 2.0, 255.0, 255.0, 0.0)

#define RC_PINK2	359.0 / 2.0
#define RC_PINK2_LOW	cvScalar(300.0 / 2.0, 90.0, 90.0, 0.0)
#define RC_PINK2_HIGH	cvScalar(359.0 / 2.0, 255.0, 255.0, 0.0)

/* Yellow modified for better filtering*/
#define RC_YELLOW	68/2
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
