#pragma once

#include "Eigen/Dense"
#include "../Inputs/SimulationInput.h"
#include <cmath>
#include "thread"
#include "future"

// 3rd October 2022, OptimumG, Pedro Calorio
// This class contains functions that implement different types of numerical integrators.
// For the moment we only have Newmark-Beta, but if more would be implemented here is the place

class Integrators {
public:
  
  // Newmark-beta main function, it is called to solve a transient simulation
  // INPUTS:
  // - forcesVector : the vector of the forces and moments for that given equations of motion
  // - massMatrix   : mass matrix of that given system of differential equations, it is assumed to be constant
  // - stopTime     : time in which the integration is no longer going to progress
  // - maxStepSize  : the maximum deltaT that the integration allows
  // - errorTol     : error tolerance for the variable step size adjustment
  // OUTPUTS:
  // - q            : Matrix of all the generalized coordinates
  // - qDot         : Matrix of all the generalized velocities
  // - qDDot        : Matrix of all the generalized accelerations
  // - time         : Resultant time vector of the integration
  
  static void newmark_solve(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector,
      const Eigen::MatrixXd& massMatrix,
      Eigen::MatrixXd& q,
      Eigen::MatrixXd& qDot,
      Eigen::MatrixXd& qDDot,
      Eigen::VectorXd& time,
      double stopTime,
      double maxStepSize,
      double errorTol);
  
  // Internal function to calculate the stiffness matrix which is the jacobian of the forces and moments vector wrt to the generalized coordinates
  // This function is internally used by newmark_solve()
  static Eigen::MatrixXd get_stiffness_matrix(const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector);
  
  // Internal function to calculate the damping matrix which is the jacobian of the forces and moments vector wrt to the generalized velocities
  // This function is internally used by newmark_solve()
  static Eigen::MatrixXd get_damping_matrix(const double time,
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector);
  
  // Internal function to calculate delta Q which is the Newton search direction
  // This function is internally used by newmark_solve()
  static Eigen::VectorXd calculate_deltaQ(const Eigen::MatrixXd& s,
      const Eigen::VectorXd& residual);
  

};

