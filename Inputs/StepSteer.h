//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_STEPSTEER_H
#define INC_4WM_STEPSTEER_H

#include "SteeringInput.h"

class StepSteer : public SteeringInput{

private:
  double _stepStart, _stepAmplitude;
  
public:
  StepSteer(double _stepStart, double _stepAmplitude);
  
  ~StepSteer() override = default;
  
  void generate_steering_profile(double simTime) override;
  
  
  
};

#endif //INC_4WM_STEPSTEER_H
