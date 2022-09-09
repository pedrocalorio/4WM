//
// Created by PedroCalorio on 8/30/2022.
//

#include "Aerodynamics.h"

Aerodynamics::Aerodynamics(double _liftCoef, double _dragCoef, double _frontArea, double _airDensity,
    double _aerobalance)
    :_liftCoef(_liftCoef), _dragCoef(_dragCoef), _frontArea(_frontArea), _airDensity(_airDensity),
     _aerobalance(_aerobalance) { }

void Aerodynamics::set_lift_coef_(double _liftCoef)
{
  Aerodynamics::_liftCoef = _liftCoef;
}

void Aerodynamics::set_drag_coef_(double _dragCoef)
{
  Aerodynamics::_dragCoef = _dragCoef;
}

void Aerodynamics::set_front_area_(double _frontArea)
{
  Aerodynamics::_frontArea = _frontArea;
}

void Aerodynamics::set_air_density_(double _airDensity)
{
  Aerodynamics::_airDensity = _airDensity;
}

void Aerodynamics::set_aerobalance_(double _aerobalance)
{
  Aerodynamics::_aerobalance = _aerobalance;
}

double Aerodynamics::get_lift_coef_() const
{
  return _liftCoef;
}

double Aerodynamics::get_drag_coef_() const
{
  return _dragCoef;
}

double Aerodynamics::get_front_area_() const
{
  return _frontArea;
}

double Aerodynamics::get_air_density_() const
{
  return _airDensity;
}

double Aerodynamics::get_aerobalance_() const
{
  return _aerobalance;
}

double Aerodynamics::get_drag_force(const double vx) const
{
  return 0.5*_airDensity*_dragCoef*_frontArea*vx*vx;
}

