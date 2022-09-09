//
// Created by PedroCalorio on 9/8/2022.
//

#include "SimulationInput.h"

SimulationInput::SimulationInput(const std::shared_ptr<LongitudinalControlInput>& longitudinal_control_input,
    const std::shared_ptr<SteeringInput>& delta)
    :longitudinal_control_input(longitudinal_control_input), delta(delta) { }

std::shared_ptr<LongitudinalControlInput>& SimulationInput::get_longitudinal_control_input_()
{
  return longitudinal_control_input;
}

std::shared_ptr<SteeringInput>& SimulationInput::get_delta_()
{
  return delta;
}
