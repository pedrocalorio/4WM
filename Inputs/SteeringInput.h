//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_STEERINGINPUT_H
#define INC_4WM_STEERINGINPUT_H


#include "../Analysis/internal_solvers/Interpolators.h"

enum class TypeSteering{
  step, ramp, sine, chirp, custom
};

class SteeringInput {

protected:
  
  Eigen::VectorXd _timeData{};
  
  Eigen::VectorXd _steeringData{};
  
  TypeSteering _typeSteering{};
  
public:
  
  virtual ~SteeringInput() = default;
  
  [[nodiscard]] double get_steering_value(double time) const;
  
  virtual void generate_steering_profile(double simTime) = 0;
  
  [[nodiscard]] const Eigen::VectorXd& get_time_data_() const;
  
  [[nodiscard]] const Eigen::VectorXd& get_steering_data_() const;
  
  
};

#endif //INC_4WM_STEERINGINPUT_H
