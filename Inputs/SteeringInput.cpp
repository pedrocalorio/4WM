//
// Created by PedroCalorio on 8/30/2022.
//

#include "SteeringInput.h"

const Eigen::VectorXd& SteeringInput::get_time_data_() const
{
  return _timeData;
}

const Eigen::VectorXd& SteeringInput::get_steering_data_() const
{
  return _steeringData;
}

double SteeringInput::get_steering_value(double time) const
{
  double yOut, slopeOut;
  
  Interpolators::interpolate(_timeData,_steeringData,time,yOut,slopeOut,InterpolationScheme::linear);
  
  return yOut;
}
