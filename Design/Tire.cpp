//
// Created by PedroCalorio on 8/30/2022.
//

#include <cmath>
#include "Tire.h"



void Tire::set_inertia_(double _inertia)
{
  Tire::_inertia = _inertia;
}

void Tire::set_corner_stiffness_(double _cornerStiffness)
{
  Tire::_cornerStiffness = _cornerStiffness;
}

void Tire::set_slip_stiffness_(double _slipStiffness)
{
  Tire::_slipStiffness = _slipStiffness;
}

double Tire::get_inertia_() const
{
  return _inertia;
}

double Tire::get_corner_stiffness_() const
{
  return _cornerStiffness;
}

double Tire::get_slip_stiffness_() const
{
  return _slipStiffness;
}

double Tire::get_loaded_radius_() const
{
  return _loadedRadius;
}

double Tire::get_lateral_force(double alpha) const
{
  return _cornerStiffness*alpha;
}

double Tire::get_longitudinal_force(double kappa) const
{
  return _slipStiffness*kappa;
}

double Tire::get_alpha(double vsy, double vsx) const
{
  return -atan(vsy/vsx);
}

double Tire::get_kappa(double vsx, double omega) const
{
  auto Rl = this->_loadedRadius;
  return -( 1 - (Rl*omega/vsx) );
}

Tire::Tire(double _inertia, double _loadedRadius, double _cornerStiffness, double _slipStiffness)
    :_inertia(_inertia), _loadedRadius(_loadedRadius), _cornerStiffness(_cornerStiffness),
     _slipStiffness(_slipStiffness) { }

void Tire::get_tire_forces(double alpha, double kappa, double& fx, double& fy) const
{
  fx = get_longitudinal_force(kappa);
  fy = get_lateral_force(alpha);
}




