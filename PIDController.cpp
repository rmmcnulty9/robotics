#include "PIDController.h"



PIDController::PIDController(double iMaxStart, double iMinStart, double iGainStart, double pGainStart, double dGainStart){
  iMax = iMaxStart;
  iMin = iMinStart;
  iGain = iGainStart;
  pGain = pGainStart;
  dGain = dGainStart;
  
  dState = 0;
  iState = 0;
}

double PIDController::UpdatePID(double error, double position)
{
  double pTerm,
 dTerm, iTerm;
  pTerm = pGain * error;   
  // calculate the proportional term
// calculate the integral state with appropriate limiting
  iState += error;
  if (iState > iMax) iState = iMax;
  else if (iState < iMin) iState = iMin;
  iTerm = iGain * iState;  // calculate the integral term
  dTerm = dGain * (position - dState);
  dState = position;
  return pTerm + iTerm - dTerm;
}