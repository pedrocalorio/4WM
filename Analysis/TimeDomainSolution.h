#pragma once

#include "Eigen/Dense"
#include <memory>

#include "../Inputs/SteeringInput.h"
#include "../Design/VehicleSetup.h"
#include "../Inputs/SimulationInput.h"
//#include "../tools/math_operations/MathOperations.h"


enum class SimulationChannel {
  longitudinal_displacement,
  longitudinal_velocity,
  longitudinal_acceleration,
  lateral_displacement,
  lateral_velocity,
  lateral_acceleration,
  yaw_velocity,
  yaw_acceleration,
  time,
  steering,
  none
};

enum class OperationType1 {
  average,
  rms,
  max,
  max_continuous,
  min,
  max_overshoot
};

enum class OperationType2 {
  none,
  ratio,
  psd,
  cpsd
};

class TimeDomainSolution {
private:
  Eigen::VectorXd m_Time;
  
  Eigen::MatrixXd m_Q, m_QDot, m_QDDot;
  
  std::shared_ptr<VehicleSetup> m_Vehicle;
  
  std::shared_ptr<SimulationInput> m_Input;

public:
  
  // ====================== CONSTRUCTOR AND DESTRUCTOR ======================== //
  
  TimeDomainSolution(const Eigen::VectorXd& time,
      const Eigen::MatrixXd& q,
      const Eigen::MatrixXd& qDot,
      const Eigen::MatrixXd& qdDot,
      const std::shared_ptr<VehicleSetup>& vehicle,
      const std::shared_ptr<SimulationInput>& input)
      :m_Time(std::move(time)), m_Q(std::move(q)), m_QDot(std::move(qDot)), m_QDDot(std::move(qdDot)),
       m_Vehicle(std::move(vehicle)), m_Input(std::move(input)) { }
  
  ~TimeDomainSolution() = default;
  
  // ======================= OPERATORS AND ASSIGNMENTS ======================== //
  
  TimeDomainSolution(const TimeDomainSolution& other)
      :m_Time(other.m_Time), m_Q(other.m_Q), m_QDot(other.m_QDot), m_QDDot(other.m_QDDot), m_Vehicle(other.m_Vehicle),
       m_Input(other.m_Input) { }
  
  TimeDomainSolution(TimeDomainSolution&& other) = delete;
  
  TimeDomainSolution& operator=(const TimeDomainSolution& other) = delete;
  
  TimeDomainSolution& operator=(TimeDomainSolution&& other) = delete;
  
  // ================================ METHODS ================================= //
  
  /// @brief This method return the requested channel in an Eigen::VectorXd
  [[nodiscard]] Eigen::VectorXd get_channel_signal(SimulationChannel channel);
};

