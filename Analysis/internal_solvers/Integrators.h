#pragma once

#include "Eigen/Dense"
#include <cmath>

class Integrators {
public:
  static void newmark_solve(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& massMatrix,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& dampingMatrix,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& stiffnessMatrix,
      Eigen::MatrixXd& q,
      Eigen::MatrixXd& qDot,
      Eigen::MatrixXd& qDDot,
      Eigen::VectorXd& time,
      double stopTime,
      double maxStepSize,
      double errorTol);
  
  
  // could not make generic so that damping and stiffness matrices are calculated internally because i would need to pass something very specific for the simulation case which is
  // the input struct, and this changes from application to application.
  

};

