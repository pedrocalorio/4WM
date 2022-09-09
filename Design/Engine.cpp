//
// Created by PedroCalorio on 9/8/2022.
//

#include "Engine.h"

Engine::Engine(double maximum_power, double gear_ratio, double power_drag)
    :maximum_power(maximum_power*745.7), gear_ratio(gear_ratio), power_drag(power_drag*745.7) { }

double Engine::get_maximum_power_() const
{
  return maximum_power;
}

void Engine::set_maximum_power_(double maximum_power)
{
  Engine::maximum_power = maximum_power;
}

double Engine::get_gear_ratio_() const
{
  return gear_ratio;
}

void Engine::set_gear_ratio_(double gear_ratio)
{
  Engine::gear_ratio = gear_ratio;
}

double Engine::get_power_drag_() const
{
  return power_drag;
}

void Engine::set_power_drag_(double power_drag)
{
  Engine::power_drag = power_drag;
}
