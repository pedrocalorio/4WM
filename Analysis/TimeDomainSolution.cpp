#include "TimeDomainSolution.h"


Eigen::VectorXd TimeDomainSolution::get_channel_signal(SimulationChannel channel)
{
  switch (channel) {
  
  case SimulationChannel::longitudinal_velocity: {
    return (Eigen::VectorXd)m_QDot.col(0).array();
  }

  case SimulationChannel::lateral_velocity: {
    return (Eigen::VectorXd)m_QDot.col(1).array();
  }

  case SimulationChannel::longitudinal_acceleration: {
    return (Eigen::VectorXd)m_QDDot.col(0).array();
  }

  case SimulationChannel::lateral_acceleration: {
    return (Eigen::VectorXd)m_QDDot.col(1).array();
  }

  case SimulationChannel::yaw_velocity: {
    return (Eigen::VectorXd)m_QDot.col(2).array();
  }

  case SimulationChannel::yaw_acceleration: {
    return (Eigen::VectorXd)m_QDDot.col(2).array();
  }

  case SimulationChannel::time: {
    return m_Time;
  }
  
  default:return m_Time;
  
  }

}