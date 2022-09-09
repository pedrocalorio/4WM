//
// Created by PedroCalorio on 8/30/2022.
//

#include "SimulationFactory.h"

std::shared_ptr<SteeringInput> SimulationFactory::create_step_input(double _stepStart, double _stepAmplitude)
{
  return std::make_shared<StepSteer>(_stepStart,_stepAmplitude);
}

std::shared_ptr<LongitudinalControlInput> SimulationFactory::create_tau_input(const Eigen::VectorXd& _timeData,
    const Eigen::VectorXd& _throttle, const Eigen::VectorXd& _brakes)
{
  return std::make_shared<LongitudinalControlInput>(_timeData,_throttle,_brakes);
}

std::shared_ptr<SteeringInput> SimulationFactory::create_custom_steering(const Eigen::VectorXd& _timeData,
    const Eigen::VectorXd& _steering)
{
  return std::make_shared<CustomSteer>(_timeData,_steering);
}

std::shared_ptr<SimulationInput> SimulationFactory::create_simulation_input(
    const std::shared_ptr<SteeringInput>& steeringInput, const std::shared_ptr<LongitudinalControlInput>& tauInput)
{
  return std::make_shared<SimulationInput>(tauInput,steeringInput);
}


