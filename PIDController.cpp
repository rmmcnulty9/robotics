#include "PIDController.h"

PIDController::PIDController(){

}

double PIDController::UpdatePID(SPid * pid, double error, double position)
{
  double pTerm,
 dTerm, iTerm;
  pTerm = pid->pGain * error;   
  // calculate the proportional term
// calculate the integral state with appropriate limiting
  pid->iState += error;
  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid->iState 
<
 pid->iMin) pid->iState = pid->iMin;
  iTerm = pid->iGain * iState;  // calculate the integral term
  dTerm = pid->dGain * (position - pid->dState);
  pid->dState = position;
  return pTerm + iTerm - dTerm;
}