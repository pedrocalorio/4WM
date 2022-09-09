//
// Created by PedroCalorio on 9/8/2022.
//

#ifndef INC_4WM_LONGITUDINALCONTROLINPUT_H
#define INC_4WM_LONGITUDINALCONTROLINPUT_H

#include "Eigen/Dense"
#include "../Analysis/internal_solvers/Interpolators.h"

class LongitudinalControlInput {

protected:
  
  Eigen::VectorXd _timeData{};
  
  Eigen::VectorXd _tauPositive{};
  
  Eigen::VectorXd _tauNegative{};
  
public:
  
  LongitudinalControlInput(const Eigen::VectorXd& _timeData, const Eigen::VectorXd& _tauPositive,
      const Eigen::VectorXd& _tauNegative);
  
  [[nodiscard]] double get_positive_tau_value(double time) const;
  
  [[nodiscard]] double get_negative_tau_value(double time) const;
  
  const Eigen::VectorXd& get_time_data_() const;
  
  void set_time_data_(const Eigen::VectorXd& _timeData);
  
  const Eigen::VectorXd& get_tau_positive_() const;
  
  void set_tau_positive_(const Eigen::VectorXd& _tauPositive);
  
  const Eigen::VectorXd& get_tau_negative_() const;
  
  void set_tau_negative_(const Eigen::VectorXd& _tauNegative);
  
};

#endif //INC_4WM_LONGITUDINALCONTROLINPUT_H
