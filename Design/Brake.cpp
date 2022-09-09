//
// Created by PedroCalorio on 9/8/2022.
//

#include "Brake.h"

Brake::Brake(double brake_bias, double maximum_torque)
    :brake_bias(brake_bias), maximum_torque(maximum_torque) { }

double Brake::get_brake_bias_() const
{
  return brake_bias;
}

void Brake::set_brake_bias_(double brake_bias)
{
  Brake::brake_bias = brake_bias;
}

double Brake::get_maximum_torque_() const
{
  return maximum_torque;
}

void Brake::set_maximum_torque_(double maximum_torque)
{
  Brake::maximum_torque = maximum_torque;
}
