#pragma once

#include "Eigen/Dense"

namespace QuarterCarSolver::Analysis
{
  class RootFinding
  {
  public:
    static [[nodiscard]] Eigen::VectorXd modified_newton_solve(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& resFunction,
        const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& jacobian,
        const Eigen::VectorXd&                                        x0,
        double                                                        initialStepSize,
        double                                                        maxStepSize,
        int                                                           maxIteration,
        double                                                        residualTol,
        bool&                                                         isSuccessful);
  };
}
