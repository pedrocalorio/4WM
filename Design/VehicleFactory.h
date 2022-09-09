//
// Created by PedroCalorio on 8/31/2022.
//

#ifndef INC_4WM_VEHICLEFACTORY_H
#define INC_4WM_VEHICLEFACTORY_H

#include <memory>

#include "Eigen/Dense"

//#include "Chassis.h"
//#include "Aerodynamics.h"
//#include "Tire.h"
#include "VehicleSetup.h"

class VehicleFactory {

public:
  static std::shared_ptr<Chassis> create_chassis(double _mass,
      double _yawInertia,
      double _weightDistribution,
      double _wheelbase,
      double _frontTrack,
      double _rearTrack);
  
  static std::shared_ptr<Aerodynamics> create_aerodynamics(double _liftCoef,
      double _dragCoef,
      double _frontArea,
      double _airDensity,
      double _aerobalance);
  
  static std::shared_ptr<Tire> create_tire(double _inertia,
      double _loadedRadius,
      double _cornerStiffness,
      double _slipStiffness);
  
  static std::shared_ptr<Brake> create_brake(double bias,
      double maximum_torque);
  
  static std::shared_ptr<Engine> create_engine(double maximum_power,
      double gear_ratio,
      double power_drag);
  
  static std::shared_ptr<VehicleSetup> create_vehicle_setup(std::shared_ptr<Chassis>& _chassis,
      std::shared_ptr<Aerodynamics>& _aero,
      std::shared_ptr<Tire>& _tire1,
      std::shared_ptr<Tire>& _tire2,
      std::shared_ptr<Tire>& _tire3,
      std::shared_ptr<Tire>& _tire4,
      std::shared_ptr<Brake>& _brakes,
      std::shared_ptr<Engine>& _engine);

};

#endif //INC_4WM_VEHICLEFACTORY_H
