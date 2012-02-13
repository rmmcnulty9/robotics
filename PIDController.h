
#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

#include "shared_constants.h"

typedef struct {
  float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
  int next_sample;
  float samples[30];
  int TAPS;
} filter;


class PIDController {
  public:
  PIDController();
  ~PIDController();
  
  updatePID();
 }
#endif
