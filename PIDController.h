
#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

#include "shared_constants.h"

typedef struct
{
  double dState;      	// Last position input
  double iState;      	// Integrator state
  double iMax, iMin;  	
  // Maximum and minimum allowable integrator state
  double	iGain,    	// integral gain
        	pGain,    	// proportional gain
         	dGain;     	// derivative gain
} SPid;


class PIDController {
  public:
  PIDController();
  ~PIDController();
  
  updatePID();
 }
#endif
