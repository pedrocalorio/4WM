//
// Created by PedroCalorio on 8/30/2022.
//

#include "StepSteer.h"

StepSteer::StepSteer(double _stepStart, double _stepAmplitude)
    :_stepStart(_stepStart), _stepAmplitude(_stepAmplitude) { _typeSteering = TypeSteering::step; }

void StepSteer::generate_steering_profile(double simTime)
{
  _timeData.resize(4);
  _steeringData.resize(4);
  
  double stepStart = _stepStart;
  
  _timeData[0] = 0;
  _timeData[1] = stepStart;
  _timeData[2] = stepStart * 1.01;
  _timeData[3] = simTime;
  
  
  _steeringData[0] = 0;
  _steeringData[1] = 0;
  _steeringData[2] = _stepAmplitude;
  _steeringData[3] = _stepAmplitude;
  
}


