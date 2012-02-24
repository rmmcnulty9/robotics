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
PIDController::~PIDController(){
}

double PIDController::UpdatePID(double error, double position)
{
	double pTerm, dTerm, iTerm;
	// calculate the proportional term
	pTerm = pGain * error;   

	// calculate the integral state with appropriate limiting
	iState += error;
	if (iState > iMax) iState = iMax;
	else if (iState < iMin) iState = iMin;
  
	// calculate the integral term
	iTerm = iGain * iState; 
	dTerm = dGain * (position - dState);
	dState = position;
	return pTerm + iTerm - dTerm;
}
