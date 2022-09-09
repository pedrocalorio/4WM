#include "RootFinding.h"

namespace QuarterCarSolver::Analysis
{
  Eigen::VectorXd RootFinding::modified_newton_solve(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& resFunction,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& jacobian,
      const Eigen::VectorXd&                                        x0,
      const double                                                  initialStepSize,
      const double                                                  maxStepSize,
      const int                                                     maxIteration,
      const double                                                  residualTol,
      bool&                                                         isSuccessful)
  {
    // Step size gain based on the initial step-size value
    auto const stepSizeGain = initialStepSize * pow(resFunction(x0).lpNorm<2>(), 2);

    // Newton Search Direction
    Eigen::VectorXd s(x0.size());
    s.setZero();

    // Newton step size
    double stepSize = initialStepSize;

    // Jacobian matrix
    Eigen::MatrixXd j(jacobian(x0).rows(), jacobian(x0).cols());
    j.setZero();

    // Residual function vector
    Eigen::VectorXd f(resFunction(x0).rows());
    f.setZero();

    // Vector of v
    Eigen::VectorXd v = Eigen::VectorXd::Ones(f.rows()); // For simplicity, all values of v are assumed as 1

    // The diagonal(v_i*f_i(x_k)) matrix
    Eigen::MatrixXd vF(j.rows(), j.cols());
    vF.setZero();

    // Current solution
    auto iteration = 0;

    auto x = x0;

    isSuccessful = false;

    while (true) {

      // Increment the iteration number
      iteration++;

      j = jacobian(x);
      f = resFunction(x);

      // Stopping criteria
      // resFunction(x).lpNorm<2>()  is the Euclidean norm of the the residual function,
      // thus it is used as the measure for the solution accuracy.
      // This condition exists the loop if the initial guess is the solution.
      if (f.lpNorm<2>() < residualTol) {
        isSuccessful = true;
        break;
      }
      // Set isSuccessful as false as the error has not been satisfied and
      // the maximum number of iterations has reached if:
      if (iteration > maxIteration) {
        isSuccessful = false;
        break;
      }

      // Calculate the diagonal matrix of v * F or diagonal(v_i*f_i(x_k))
      // v.array() * F.array() is the element-wise multiplication
      vF = ( v.array() * f.array() ).matrix().asDiagonal();

      // Assign the sign of df_i(x)/dx_i for to v_i*f_i
      for (unsigned i = 0 ; i < vF.rows() ; i++) {
        vF(i, i) = std::copysign(vF(i, i), j(i, i));
      }

      // Calculate the search direction vector (regular Newton_Raphson)
      // s = -jacobian(x).inverse() * resFunction(x);

      // Calculate the search direction vector
      // v.array() + J.array() is the element-wise summation
      s = -( ( vF.array() + j.array() ).matrix().inverse() ) * f; // change this to Eigen solvers

      // Damped Newton-Raphson Step-Size:
      // Determine the step size based on the value of the residual's Euclidean norm^(-2):
      // At first iterations, where x is far from the solution, the norm will have a large value, thus
      // stepSizeGain*||f(x)||^-2 will be << 1 and the step size to go, however, as x converges to the solution,
      // the residual norm will be smaller and gamma*||f(x)||^-2 will be >> 1 and the maxStepSize will be the step size to go.
      stepSize = std::min(maxStepSize, stepSizeGain * pow(f.lpNorm<2>(), -2));

      // Update the solution
      x = x + stepSize * s;

    }
    return x;
  }
}
