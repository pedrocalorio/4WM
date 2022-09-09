//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_SIMULATIONFACTORY_H
#define INC_4WM_SIMULATIONFACTORY_H

#include "SteeringInput.h"
#include "StepSteer.h"
#include "LongitudinalControlInput.h"
#include "SimulationInput.h"
#include "CustomSteer.h"

#include <memory>

class SimulationFactory {
public:
  // methods
  static std::shared_ptr<SimulationInput> create_simulation_input(const std::shared_ptr<SteeringInput>& steeringInput,
      const std::shared_ptr<LongitudinalControlInput>& tauInput);
  
  static std::shared_ptr<SteeringInput> create_step_input(double _stepStart, double _stepAmplitude);
  
  static std::shared_ptr<LongitudinalControlInput> create_tau_input(const Eigen::VectorXd& _timeData,
      const Eigen::VectorXd& _throttle,
      const Eigen::VectorXd& _brakes);
  
  static std::shared_ptr<SteeringInput> create_custom_steering(const Eigen::VectorXd& _timeData,
      const Eigen::VectorXd& _steering);

};

#endif //INC_4WM_SIMULATIONFACTORY_H
