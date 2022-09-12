cmake_minimum_required(VERSION 3.0)
project(4WM)

set(CMAKE_CXX_STANDARD 20)

set(EIGEN_DIR "C:/Users/PedroCalorio/CLionProjects/4WM/Dependancies/eigen-3.4.0")
include_directories(${EIGEN_DIR})

set(SOLVERS_DIR "C:/Users/PedroCalorio/CLionProjects/4WM/Analysis")
include_directories(${SOLVERS_DIR})

set(TOOLS_DIR "C:/Users/PedroCalorio/CLionProjects/4WM/Tools/data_stream")
include_directories(${TOOLS_DIR})

add_executable(4WM main.cpp
               Design/Chassis.cpp
               Design/Chassis.h
               Design/Aerodynamics.cpp
               Design/Aerodynamics.h
               Design/Tire.cpp
               Design/Tire.h
               Inputs/SteeringInput.cpp
               Inputs/SteeringInput.h
               Inputs/StepSteer.cpp
               Inputs/StepSteer.h
               Inputs/SimulationFactory.cpp
               Inputs/SimulationFactory.h
               Design/VehicleSetup.cpp
               Design/VehicleSetup.h
               Design/VehicleFactory.cpp
               Design/VehicleFactory.h
               Analysis/SolverSettings.h
               Analysis/SolverSettings.cpp
               Analysis/SolversFactory.h
               Analysis/SolversFactory.cpp
               Analysis/TimeDomainSolution.h
               Analysis/TimeDomainSolution.cpp
               Analysis/internal_solvers/Integrators.h
               Analysis/internal_solvers/Integrators.cpp
               Analysis/internal_solvers/Interpolators.h
               Analysis/internal_solvers/Interpolators.cpp
               Analysis/internal_solvers/RootFinding.h
               Analysis/internal_solvers/RootFinding.cpp
               Inputs/LongitudinalControlInput.cpp
               Inputs/LongitudinalControlInput.h
               Inputs/SimulationInput.cpp
               Inputs/SimulationInput.h
               Inputs/CustomSteer.cpp
               Inputs/CustomSteer.h
               Design/Brake.cpp
               Design/Brake.h
               Design/Engine.cpp
               Design/Engine.h
               Tools/data_stream/DataStream.h
               Tools/data_stream/DataStream.cpp)

