//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_VEHICLESETUP_H
#define INC_4WM_VEHICLESETUP_H

#include "Chassis.h"
#include "Aerodynamics.h"
#include "Tire.h"
#include "Engine.h"
#include "Brake.h"
#include "../Inputs/SteeringInput.h"
#include "../Inputs/LongitudinalControlInput.h"
#include "../Inputs/SimulationInput.h"

class VehicleSetup {
  
  friend class SimulationInput;

protected:
  
  std::shared_ptr<Chassis> _chassis{};
  
  std::shared_ptr<Aerodynamics> _aerodynamics{};
  
  std::shared_ptr<Engine> _engine{};
  
  std::shared_ptr<Brake> _brake{};
  
  std::shared_ptr<Tire> _tire1{};
  
  std::shared_ptr<Tire> _tire2{};
  
  std::shared_ptr<Tire> _tire3{};
  
  std::shared_ptr<Tire> _tire4{};

public:
 
  VehicleSetup(const std::shared_ptr<Chassis>& _chassis, const std::shared_ptr<Aerodynamics>& _aerodynamics,
      const std::shared_ptr<Engine>& _engine, const std::shared_ptr<Brake>& _brake, const std::shared_ptr<Tire>& _tire1,
      const std::shared_ptr<Tire>& _tire2, const std::shared_ptr<Tire>& _tire3, const std::shared_ptr<Tire>& _tire4);
  
  
  //methods
  
  Eigen::MatrixXd get_mass_matrix();
  
  void get_wheels_torque(const double& tau_pos,
      const double& tau_neg,
      const Eigen::VectorXd& states,
      Eigen::Vector4d& wheel_torques);
  
  Eigen::VectorXd get_forces_and_moments_vector(const std::shared_ptr<SimulationInput>& steerInput,
      const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot);

  Eigen::MatrixXd get_stiffness_matrix(const std::shared_ptr<SimulationInput>& input,
      const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot);
  
  Eigen::MatrixXd get_damping_matrix(const std::shared_ptr<SimulationInput>& input,
      const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot);
  
  
  
  
  
};

#endif //INC_4WM_VEHICLESETUP_H
