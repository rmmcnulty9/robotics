/*
 * PIDController Header
 *  Used for the PID controller
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

class PIDController {
  public:
  PIDController(double iMaxStart, double iMinStart, double iGainStart, double pGainStart, double dGainStart);
  ~PIDController();
  
  double UpdatePID(double error, double position);
  
private:
  
    double dState;      	// Last position input
  double iState;      	// Integrator state
  
    double iMax, iMin;  	
  // Maximum and minimum allowable integrator state
  double	iGain,    	// integral gain
        	pGain,    	// proportional gain
         	dGain;     	// derivative gain
 };
#endif
