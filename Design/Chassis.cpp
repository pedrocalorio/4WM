//
// Created by PedroCalorio on 8/30/2022.
//

#include "Chassis.h"

Chassis::Chassis(double _mass, double _yawInertia, double _weightDistribution, double _wheelbase, double _frontTrack,
    double _rearTrack)
    :_mass(_mass), _yawInertia(_yawInertia), _weightDistribution(_weightDistribution), _wheelbase(_wheelbase),
     _frontTrack(_frontTrack), _rearTrack(_rearTrack) { }

Chassis::Chassis() {
  // default values for LMP2 car
  _mass=1000;
  _yawInertia=1250;
  _frontTrack=1.2;
  _rearTrack=1.2;
  _wheelbase=3.0;
  _weightDistribution=0.48;
}

void Chassis::set_mass_(double _mass)
{
  Chassis::_mass = _mass;
}

void Chassis::set_yaw_inertia_(double _yawInertia)
{
  Chassis::_yawInertia = _yawInertia;
}

void Chassis::set_weight_distribution_(double _weightDistribution)
{
  Chassis::_weightDistribution = _weightDistribution;
}

void Chassis::set_wheelbase_(double _wheelbase)
{
  Chassis::_wheelbase = _wheelbase;
}

void Chassis::set_front_track_(double _frontTrack)
{
  Chassis::_frontTrack = _frontTrack;
}

void Chassis::set_rear_track_(double _rearTrack)
{
  Chassis::_rearTrack = _rearTrack;
}

double Chassis::get_mass_() const
{
  return _mass;
}

double Chassis::get_yaw_inertia_() const
{
  return _yawInertia;
}

double Chassis::get_weight_distribution_() const
{
  return _weightDistribution;
}

double Chassis::get_wheelbase_() const
{
  return _wheelbase;
}

double Chassis::get_front_track_() const
{
  return _frontTrack;
}

double Chassis::get_rear_track_() const
{
  return _rearTrack;
}
