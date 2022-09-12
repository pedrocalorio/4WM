#pragma once

#include "Eigen/Dense"
#include "../Inputs/SimulationInput.h"
#include <cmath>
#include "thread"

class Integrators {
public:
//  static void newmark_solve(
//      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector,
//      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& massMatrix,
//      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& dampingMatrix,
//      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& stiffnessMatrix,
//      Eigen::MatrixXd& q,
//      Eigen::MatrixXd& qDot,
//      Eigen::MatrixXd& qDDot,
//      Eigen::VectorXd& time,
//      double stopTime,
//      double maxStepSize,
//      double errorTol);
  
  static void newmark_solve(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector,
      const Eigen::MatrixXd& massMatrix,
      const std::shared_ptr<SimulationInput>& input,
      Eigen::MatrixXd& q,
      Eigen::MatrixXd& qDot,
      Eigen::MatrixXd& qDDot,
      Eigen::VectorXd& time,
      double stopTime,
      double maxStepSize,
      double errorTol);
  
  
  // could not make generic so that damping and stiffness matrices are calculated internally because i would need to pass something very specific for the simulation case which is
  // the input struct, and this changes from application to application.
  
  static Eigen::MatrixXd get_stiffness_matrix(const std::shared_ptr<SimulationInput>& input,
      const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector);
  
  static Eigen::MatrixXd get_damping_matrix(const std::shared_ptr<SimulationInput>& input,
      const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector);
  

};

