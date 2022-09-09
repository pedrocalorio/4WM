//
// Created by PedroCalorio on 9/8/2022.
//

#ifndef INC_4WM_SIMULATIONINPUT_H
#define INC_4WM_SIMULATIONINPUT_H

#include "Eigen/Dense"
#include "LongitudinalControlInput.h"
#include "SteeringInput.h"

class SimulationInput {

public:
  
  std::shared_ptr<LongitudinalControlInput> longitudinal_control_input{};
  std::shared_ptr<SteeringInput> delta{};
  
public:
  
  SimulationInput(const std::shared_ptr<LongitudinalControlInput>& longitudinal_control_input,
      const std::shared_ptr<SteeringInput>& delta);
  
  std::shared_ptr<LongitudinalControlInput>& get_longitudinal_control_input_() ;
  
  std::shared_ptr<SteeringInput>& get_delta_() ;
  
};

#endif //INC_4WM_SIMULATIONINPUT_H
