#include "Integrators.h"

void Integrators::newmark_solve(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& forcesVector,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& massMatrix,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& dampingMatrix,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& stiffnessMatrix,
    Eigen::MatrixXd& q,
    Eigen::MatrixXd& qDot,
    Eigen::MatrixXd& qDDot,
    Eigen::VectorXd& time,
    const double stopTime,
    const double maxStepSize,
    const double errorTol)
{
  // ------------------------------------------------------------------------------------
  // Newmark - beta parameters-----------------------------------------------------------
  // ------------------------------------------------------------------------------------
  
  double const gamma{0.5};
  double const beta = pow(gamma + 0.5, 2) / 4;
  
  // ------------------------------------------------------------------------------------
  // Step-size selection parameters------------------------------------------------------
  // ------------------------------------------------------------------------------------
  
  double const safetyFactor{0.75};
  double const upperTol = 1.25 * errorTol;                   // Upper tolerance limit
  double const lowerTol = 0.75 * errorTol;                   // Lower tolerance limit
  double const stepperMaxIterations{5};
  double stepperIteration{1};
  double stepSize{maxStepSize};
  
  
  // ------------------------------------------------------------------------------------
  // Newton-Raphson parameters ----------------------------------------------------------
  // ------------------------------------------------------------------------------------
  
  // maximum number of acceptable iterations for the inner Newton
  int const newtonMaxIterations{100};
  double const newtonTol{1e-7};
  
  // ------------------------------------------------------------------------------------
  // Local variables used in Newmark iterations  ----------------------------------------
  // ------------------------------------------------------------------------------------
  //
  // Weighted Norm of acceleration
  double xNorm{1.0};
  
  // Error scale declaration
  double psi = {1.0};
  
  // unitless scaling parameter
  double theta = {1.0};
  
  // Normalization vector that holds the max(1, max (|q_ij| for j=1,...,n+1 ))
  Eigen::VectorXd y(q.cols());
  y.setOnes();
  
  // Residual vector
  Eigen::VectorXd residual(q.cols());
  residual.setOnes();
  
  // Jacobian matrix
  Eigen::MatrixXd s(q.rows(), q.rows());
  s.setOnes();
  
  // Newton search direction vector for generalized coordinates
  Eigen::VectorXd deltaQ(q.cols());
  deltaQ.setOnes();
  
  // Acceleration correction vector
  Eigen::VectorXd x(q.cols());
  x.setOnes();
  
  // Integration's local error vector
  Eigen::VectorXd delta(q.cols());
  delta.setOnes();
  
  // this boolean will allow the step-size correction to increase the
  // step size if needed. However, if the last step-size has been decreased
  // to fit the final time slot, this boolean will be set to false.
  bool allowStepSizeGrowth = true;
  // This boolean will be set to true if the error at the current step-size
  // is higher than the upper tolerance limit and will be set to false
  // if there is no need to do the previous step again.
  // This will not allow the termination condition to activate
  // unless the error tolerance is satisfied. This is specifically
  // helpful in the last step when the last element of time vector
  // is equal to the stopping time, but the error has not been satisfied yet. Therefore,
  // the final step will be repeated again until the error tolerance is satisfied.
  bool doPreviousStepAgain = true;
  
  // ------------------------------------------------------------------------------------
  // Allocate memory for q, qDot, qDDot, and time vectors--------------------------------
  // ------------------------------------------------------------------------------------
  
  // Here, we estimate the required allocated size for the vectors, however, there will be
  // a check in each iteration of Newmark to make sure this size is sufficient, otherwise, the vectors
  // will be resized again.
  
  Eigen::Index estimatedSteps = static_cast<long long>(static_cast<int>(ceil(stopTime / maxStepSize)))
      * static_cast<long long>(100);
  
  time.conservativeResize(estimatedSteps, 1);
  q.conservativeResize(estimatedSteps, q.cols());
  qDot.conservativeResize(estimatedSteps, qDot.cols());
  qDDot.conservativeResize(estimatedSteps, qDDot.cols());
  
  // ------------------------------------------------------------------------------------
  // Newmark-beta Iterations ------------------------------------------------------------
  // ------------------------------------------------------------------------------------
  
  // A row vector that holds the maximum value of the states up to the
  // current state
  Eigen::VectorXd maxQ(q.cols());
  
  // Error
  double error{100};
  
  // integration step number
  Eigen::Index n = 0;
  Eigen::Index nPlusOne = 0;
  
  // Generalized coordinates' degree of freedom
  auto const nQ = static_cast<int>(q.cols());
  
  while (true) {
    
    nPlusOne = n + static_cast<long long>(1);
    
    // Here, we check the size of the vectors to make sure we are not
    // running out of memory. Note that "conservativeResize" is an
    // expensive operation and must be used with caution!
    
    if (n >= estimatedSteps - static_cast<long long>(1)) {
      
      // Increase the current size by 1,000 elements
      estimatedSteps = estimatedSteps + static_cast<long long>(1'000);
      
      time.conservativeResize(estimatedSteps, 1);
      q.conservativeResize(estimatedSteps, q.cols());
      qDot.conservativeResize(estimatedSteps, qDot.cols());
      qDDot.conservativeResize(estimatedSteps, qDDot.cols());
    }
    
    // this condition adjusts the final step size so that the
    // the final element of the time vector is the user specified stop time.
    
    //if (time(n) + h > stopTime) {
    //  h                   = abs(stopTime - time(n));
    //  allowStepSizeGrowth = false;
    //}
    
    // Time incrementation
    time(nPlusOne) = time(n) + stepSize;
    
    // ------------------------------------------------------------------------------------
    // Prediction--------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------
    
    qDDot.row(nPlusOne).setZero();
    qDot.row(nPlusOne) = qDot.row(n) + (1 - gamma) * stepSize * qDDot.row(n);
    q.row(nPlusOne) = q.row(n) + stepSize * qDot.row(n) + (0.5 - beta) * stepSize * stepSize * qDDot.row(n);
    
    // we use .row because it corresponds to each time instant
    
    // ------------------------------------------------------------------------------------
    // Correction with Newton-raphson -----------------------------------------------------
    // ------------------------------------------------------------------------------------
    
    // Newton iteration
    long int k = 0;
    while (true) {
      
      k += 1;
      
      // Residual Vector Evaluation Mq'' + f(q',q,t) = 0
      residual = massMatrix(q.row(nPlusOne), qDot.row(nPlusOne), time(nPlusOne)) * qDDot.row(nPlusOne).transpose() +
          forcesVector(q.row(nPlusOne), qDot.row(nPlusOne), time(nPlusOne));
      
      // Convergence Check
      if (residual.lpNorm<2>() < newtonTol || k > newtonMaxIterations) {
        break;
      }
      
      // Calculation of the Jacobian matrix
      s = stiffnessMatrix(q.row(nPlusOne), qDot.row(nPlusOne), time(nPlusOne)) +
          gamma / (beta * stepSize) * dampingMatrix(q.row(nPlusOne), qDot.row(nPlusOne), time(nPlusOne)) +
          1 / (beta * stepSize * stepSize) * massMatrix(q.row(nPlusOne), qDot.row(nPlusOne), time(nPlusOne));
      
      // Calculate Newton Search Direction
//      deltaQ = -s.inverse() * residual;
      
      deltaQ = s.colPivHouseholderQr().solve(-residual);
      // There are no requirements on s matrix for colPivHouseholder to solve this with this method.
      
      // ------------------------------------------------------------------------------------
      // Correction--------------------------------------------------------------------------
      // ------------------------------------------------------------------------------------
      q.row(nPlusOne) = q.row(nPlusOne) + deltaQ.transpose();
      qDot.row(nPlusOne) = qDot.row(nPlusOne) + gamma / (beta * stepSize) * deltaQ.transpose();
      qDDot.row(nPlusOne) = qDDot.row(nPlusOne) + 1 / (beta * stepSize * stepSize) * deltaQ.transpose();
      
    }
    
    // ------------------------------------------------------------------------------------
    // Approximate the truncation error----------------------------------------------------
    // ------------------------------------------------------------------------------------
    
    x = qDDot.row(nPlusOne) - qDDot.row(n);
    
    // ------------------------------------------------------------------------------------
    // Local integration error-------------------------------------------------------------
    // ------------------------------------------------------------------------------------
    
    delta = (beta - 1.0 / 6.0) * stepSize * stepSize * x.transpose();
    
    // Normalization Factor
    for (auto i = 0; i < q.cols(); i++) {
      
      maxQ(i) = std::max(maxQ(i), abs(q(n, i)));
      
      y(i) = std::max(1.0, maxQ(i));
    }
    
    // ------------------------------------------------------------------------------------
    // Weighted norm of error -------------------------------------------------------------
    // ------------------------------------------------------------------------------------
    error = sqrt(1.0 / nQ * (delta.array().square() / y.array().square()).sum());
    
    //------------------------------------------------------------------------------------
    // Weighted norm of acceleration------------------------------------------------------
    // -----------------------------------------------------------------------------------
    
    xNorm = sqrt((x.array().square() / y.array().square()).sum());
    
    // ------------------------------------------------------------------------------------
    // Scaled user defined error tolerance ------------------------------------------------
    // ------------------------------------------------------------------------------------
    
    // Error scaling
    psi = nQ * pow(errorTol, 2) / pow((beta - 1.0 / 6.0), 2);
    
    // Unitless scaling parameter to adjust the time-step
    theta = pow(xNorm, 2) * pow(stepSize, 4) / psi;
    
    // check if the integration step-size needs to be modified
    if (error < lowerTol || upperTol < error) {
      
      // check the error condition
      if (error > upperTol) {
        
        // if the error is higher then the upper tolerance limit,
        // adjust the step size and simulate the same step (nth step) again.
        stepSize = safetyFactor * stepSize / pow(theta, 1.0 / 6.0);
        
        doPreviousStepAgain = true;
        
        // This is for when the step-size has been reduced to fit the time-gap between the current
        // time and the stopping time. However, this reduced step-size might not satisfy the
        // error tolerances, therefore, if the step size is needed to be reduced even more, the allowStepSizeGrowth
        // must be set to true in case the reduced step size is too small.
        allowStepSizeGrowth = true;
      }
      else if (error < lowerTol && allowStepSizeGrowth == true) {
        // if the error is lower than the lower tolerance limit,
        // there is no need to do the calculations for this step again.
        // proceed to the next step (n = n+1) and adjust the step size.
        
        // This condition checks whether the maximum number of allowable
        // step-size changes--when the error is lower than gammaL--is met.
        if (stepperIteration <= stepperMaxIterations) {
          
          stepSize = std::min(maxStepSize, 0.9 * stepSize / pow(theta, 1.0 / 6.0));
          
          n += 1;
          stepperIteration += 1;
          doPreviousStepAgain = false;
        }
        else {
          stepSize = maxStepSize;
          n += 1;
          doPreviousStepAgain = false;
          stepperIteration = 1;
        }
      }
    }
    else {
      // else if lowerTol < error < upperTol, then keep step-size the same and use it for the next step.
      n += 1;
      doPreviousStepAgain = false;
    }
    
    // checks to see if the last element of the time vector is the stopping time.
    // this is done only if the doPreviousStepAgain is false.
    if (time(n) >= stopTime && !doPreviousStepAgain) {
      
      // Release the extra memory allocated for the vectors:
      time.conservativeResize(nPlusOne, 1);
      q.conservativeResize(nPlusOne, q.cols());
      qDot.conservativeResize(nPlusOne, qDot.cols());
      qDDot.conservativeResize(nPlusOne, qDDot.cols());
      
      break;
    }
    
  }
}

