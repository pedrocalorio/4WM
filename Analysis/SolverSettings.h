#pragma once

class SolversFactory;

class SolverSettings {
  friend SolversFactory;

public:
  double m_SsSolverInitialStepSize;
  
  double m_SsSolverMaxStepSize;
  
  int m_SsSolverMaxIterations;
  
  double m_SsSolverTol;
  
  double m_TransientSolverMaxStepSize;
  
  double m_TransientSolverTol;
  
  double m_StartTime;
  
  double m_EndTime;

public:
  SolverSettings(const double ssSolverInitialStepSize,
      const double ssSolverMaxStepSize,
      const int ssSolverMaxIterations,
      const double ssSolverTol,
      const double transientSolverMaxStepSize,
      const double transientSolverTol,
      const double startTime, const double endTime)
      :m_SsSolverInitialStepSize(ssSolverInitialStepSize), m_SsSolverMaxStepSize(ssSolverMaxStepSize),
       m_SsSolverMaxIterations(ssSolverMaxIterations), m_SsSolverTol(ssSolverTol),
       m_TransientSolverMaxStepSize(transientSolverMaxStepSize), m_TransientSolverTol(transientSolverTol),
       m_StartTime(startTime), m_EndTime(endTime) { }
  
  ~SolverSettings() = default;
  
  
  
  SolverSettings(const SolverSettings& other) = default;
  
  SolverSettings(SolverSettings&& other) = delete;
  
  SolverSettings& operator=(const SolverSettings& other) = delete;
  
  SolverSettings& operator=(SolverSettings&& other) = delete;
  
  
  
  
};

