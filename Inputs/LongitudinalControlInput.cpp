//
// Created by PedroCalorio on 9/8/2022.
//

#include "LongitudinalControlInput.h"

const Eigen::VectorXd& LongitudinalControlInput::get_time_data_() const
{
  return _timeData;
}

void LongitudinalControlInput::set_time_data_(const Eigen::VectorXd& _timeData)
{
  LongitudinalControlInput::_timeData = _timeData;
}

const Eigen::VectorXd& LongitudinalControlInput::get_tau_positive_() const
{
  return _tauPositive;
}

void LongitudinalControlInput::set_tau_positive_(const Eigen::VectorXd& _tauPositive)
{
  LongitudinalControlInput::_tauPositive = _tauPositive;
}

const Eigen::VectorXd& LongitudinalControlInput::get_tau_negative_() const
{
  return _tauNegative;
}

void LongitudinalControlInput::set_tau_negative_(const Eigen::VectorXd& _tauNegative)
{
  LongitudinalControlInput::_tauNegative = _tauNegative;
}

LongitudinalControlInput::LongitudinalControlInput(const Eigen::VectorXd& _timeData,
    const Eigen::VectorXd& _tauPositive, const Eigen::VectorXd& _tauNegative)
    :_timeData(_timeData), _tauPositive(_tauPositive), _tauNegative(_tauNegative) { }

double LongitudinalControlInput::get_positive_tau_value(double time) const
{
  double yOut, slopeOut;
  
  Interpolators::interpolate(_timeData,_tauPositive,time,yOut,slopeOut,InterpolationScheme::linear);
  
  return yOut;
}

double LongitudinalControlInput::get_negative_tau_value(double time) const
{
  double yOut, slopeOut;
  
  Interpolators::interpolate(_timeData,_tauNegative,time,yOut,slopeOut,InterpolationScheme::linear);
  
  return yOut;
}
