#pragma once

#include <memory>
#include <vector>

#include "../Analysis/TimeDomainSolution.h"
#include "../Design/VehicleSetup.h"
#include "../Analysis/internal_solvers/RootFinding.h"
#include "../Analysis/internal_solvers/Interpolators.h"
#include "../Analysis/internal_solvers/Integrators.h"
#include "../Inputs/SteeringInput.h"
#include "../Analysis/SolverSettings.h"

class SolversFactory {
public:
  
  friend VehicleSetup;
  
  static std::shared_ptr<SolverSettings> create_solver_settings(double ssSolverInitialStepSize,
      double ssSolverMaxStepSize,
      int ssSolverMaxIterations,
      double ssSolverTol,
      double transientSolverMaxStepSize,
      double transientSolverTol,
      double startTime, double endTime);
  
  static std::shared_ptr<TimeDomainSolution>
  solve_single_time_domain_simulation(const std::shared_ptr<SimulationInput>& input,
      const std::shared_ptr<VehicleSetup>& vehicle,
      const std::shared_ptr<SolverSettings>& solverSettings);

//    static [[nodiscard]] std::vector<std::shared_ptr<TimeDomainSolution>>
//      solve_batch_time_domain_simulations(const std::shared_ptr<Roads::Road>&                        road,
//                                          const std::vector<std::shared_ptr<Design::VehicleCorner>>& vehicleCorners,
//                                          double                                                     vehicleSpeed,
//                                          const std::shared_ptr<SolverSettings>&                     solverSettings);
};
