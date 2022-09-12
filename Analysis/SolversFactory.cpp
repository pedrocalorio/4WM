#include "SolversFactory.h"

std::shared_ptr<SolverSettings> SolversFactory::create_solver_settings(double ssSolverInitialStepSize,
    double ssSolverMaxStepSize,
    int ssSolverMaxIterations,
    double ssSolverTol,
    double transientSolverMaxStepSize,
    double transientSolverTol,
    double startTime, double endTime)
{
  return std::make_shared<SolverSettings>(ssSolverInitialStepSize, ssSolverMaxStepSize, ssSolverMaxIterations,
      ssSolverTol, transientSolverMaxStepSize, transientSolverTol, startTime,
      endTime);
}

std::shared_ptr<TimeDomainSolution> SolversFactory::solve_single_time_domain_simulation(
    const std::shared_ptr<SimulationInput>& input,
    const std::shared_ptr<VehicleSetup>& vehicle,
    const std::shared_ptr<SolverSettings>& solverSettings)
{
  
//  auto steering_input = input->get_delta_();
//  auto tau_input      = input->get_longitudinal_control_input_();
//
//  steering_input->generate_steering_profile(solverSettings->m_EndTime);
//
//  input->
  
  // ------------------------------------------------------------------------------------
  // Calculate the initial solution for the steady - state analysis ---------------------
  // ------------------------------------------------------------------------------------
  
  
  
  // Vector of initial guesses for the steady-state solver.
  Eigen::VectorXd q0(7);
  q0(0) = 0;
  q0(1) = 0;
  q0(2) = 0;
  q0(3) = 0;
  q0(4) = 0;
  q0(5) = 0;
  q0(6) = 0;
  
  Eigen::VectorXd qDot0(7);
  auto vx0 = 150/3.6;
  qDot0(0) = vx0;
  qDot0(1) = 0;
  qDot0(2) = 0;
  qDot0(3) = vx0/0.330;
  qDot0(4) = vx0/0.330;
  qDot0(5) = vx0/0.330;
  qDot0(6) = vx0/0.330;
  
 
  
  // ------------------------------------------------------------------------------------
  // Solve the transient simulation via Newmark - beta ----------------------------------
  // ------------------------------------------------------------------------------------
  
  // Construct the mass matrix to pass to the numerical integrator
  Eigen::MatrixXd massMatrix = vehicle->get_mass_matrix();
  
  auto forcesLambda = [&input, &vehicle](const Eigen::VectorXd& q,
      const Eigen::VectorXd& qDot,
      const double time) {
    return vehicle->get_forces_and_moments_vector(input, time, q, qDot);
  };
  
  // Initialize the vectors
  Eigen::MatrixXd q(1, 7), qDot(1, 7), qDDot(1, 7);
  Eigen::VectorXd time(1);
  
  q.row(0) = q0;
  qDot.row(0) = qDot0;
  qDDot.setZero();
  
  time(0) = solverSettings->m_StartTime;
  
  Integrators::newmark_solve(forcesLambda, massMatrix, input,
      q, qDot, qDDot, time,
      solverSettings->m_EndTime,
      solverSettings->m_TransientSolverMaxStepSize,
      solverSettings->m_TransientSolverTol);
  
  // Create an object of TimeDomainSolution:
  
  auto solution = std::make_shared<TimeDomainSolution>(time, q, qDot, qDDot, vehicle, input);
  
  return solution;
}

//  std::vector<std::shared_ptr<TimeDomainSolution>>
//    SolversFactory::solve_batch_time_domain_simulations(
//        const std::shared_ptr<Roads::Road>&                        road,
//        const std::vector<std::shared_ptr<Design::VehicleCorner>>& vehicleCorners,
//        const double                                               vehicleSpeed,
//        const std::shared_ptr<SolverSettings>&                     solverSettings)
//  {
//    std::vector<std::shared_ptr<TimeDomainSolution>> solutions;
//
//    for (unsigned long long i = 0 ; i < vehicleCorners.size() ; i++) {
//
//      auto solution = solve_single_time_domain_simulation(road,
//                                                          vehicleCorners[i],
//                                                          vehicleSpeed,
//                                                          solverSettings);
//
//      solutions.push_back(solution);
//
//    }
//
//    return solutions;
//  }

