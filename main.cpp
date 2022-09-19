
#include <iostream>
#include "Eigen/Dense"
#include "Analysis/internal_solvers/Integrators.h"
#include "Design/Aerodynamics.h"
#include "Design/Chassis.h"
#include "Design/Tire.h"
#include "Design/VehicleSetup.h"
#include "Design/VehicleFactory.h"
#include "Inputs/StepSteer.h"
#include "Inputs/CustomSteer.h"
#include "Inputs/SteeringInput.h"
#include "Inputs/SimulationFactory.h"
#include "Analysis/SolversFactory.h"
#include "Analysis/SolverSettings.h"
#include "Analysis/TimeDomainSolution.h"
#include "Tools/data_stream/DataStream.h"
#include <fstream>
#include <chrono>
#include <thread> // sleep_for, for testing only


int main()
{
//  std::filesystem::path cwd = std::filesystem::current_path();
//  std::string file(cwd.string());

  std::string file = "C:/Users/PedroCalorio/CLionProjects/4WM/";
  
  // initialize all vehicle inputs
  auto chassis = VehicleFactory::create_chassis(1000,1250,0.48,3.00,1.50,1.55);
  auto aero = VehicleFactory::create_aerodynamics(4, 1.0, 1, 1.226, 0.44);
  auto tire_front = VehicleFactory::create_tire(3, 0.330, 8e3, 4e3);
  auto tire_rear = VehicleFactory::create_tire(3, 0.330, 10e3, 5e3);
  auto brake = VehicleFactory::create_brake(0.70,5e3);
  auto engine = VehicleFactory::create_engine(450,3.7700,0); // power in hp
  
  // create a vehicle setup
  auto vehicle_setup = VehicleFactory::create_vehicle_setup(chassis,aero,tire_front,tire_front,tire_rear,tire_rear,brake,engine);
  
  // create simulation inputs
  std::string tr_steering = file + "TrackReplayInput_Steer.csv";
  std::string tr_tps      = file + "TrackReplayInput_TPS.csv";
  std::string tr_brakes   = file + "TrackReplayInput_Brakes.csv";
  
  Eigen::VectorXd _timeData = DataStream::read_csv_column_data(tr_steering,ColumnNumber::column_one);
  Eigen::VectorXd _steerData = DataStream::read_csv_column_data(tr_steering,ColumnNumber::column_two);
  Eigen::VectorXd _tpsData = DataStream::read_csv_column_data(tr_tps,ColumnNumber::column_two);
  Eigen::VectorXd _brakesData = DataStream::read_csv_column_data(tr_brakes,ColumnNumber::column_two);
  
  auto input_steering = SimulationFactory::create_custom_steering(_timeData,_steerData);
//  auto input_steering = SimulationFactory::create_step_input(1,2/57.3);
  auto input_tau = SimulationFactory::create_tau_input(_timeData,_tpsData,_brakesData);
  
  auto input = SimulationFactory::create_simulation_input(input_steering,input_tau);
  
  // run simulation
  auto solversetting = SolversFactory::create_solver_settings(0.01, 0.1, 1000, 1e-6, 0.005, 1e-6, 0, _timeData.maxCoeff());
  
  auto start = std::chrono::high_resolution_clock::now();
  auto sol = SolversFactory::solve_single_time_domain_simulation(input,vehicle_setup,solversetting);
  auto end = std::chrono::high_resolution_clock::now();
  
  std::chrono::duration<float> duration = end - start;
  std::cout << "simulation ran in " << duration.count() << " seconds." << std::endl;
  
  // get the outputs
  
  auto yawrate= sol->get_channel_signal(SimulationChannel::yaw_velocity);
  auto time= sol->get_channel_signal(SimulationChannel::time);
  
  std::ofstream output;
  output.open("C:/Users/PedroCalorio/CLionProjects/4WM/test.csv");
  
  output << "t (s)" << "," << "r (rad/s)" << "\n";
  
  for (unsigned int i=0; i < sol->get_channel_signal(SimulationChannel::time).size(); i++){
    output << time(i) << "," << yawrate(i) << "\n";
  }
  
  output.close();
  
}

