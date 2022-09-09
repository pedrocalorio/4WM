//
// Created by PedroCalorio on 8/31/2022.
//

#include "VehicleFactory.h"

std::shared_ptr<Chassis> VehicleFactory::create_chassis(double _mass, double _yawInertia, double _weightDistribution,
    double _wheelbase, double _frontTrack, double _rearTrack)
{
  return std::make_shared<Chassis>(_mass,_yawInertia,_weightDistribution,_wheelbase,_frontTrack,_rearTrack);
}

std::shared_ptr<Aerodynamics> VehicleFactory::create_aerodynamics(double _liftCoef, double _dragCoef, double _frontArea,
    double _airDensity, double _aerobalance)
{
  return std::make_shared<Aerodynamics>(_liftCoef,_dragCoef,_frontArea,_airDensity,_aerobalance);
}

std::shared_ptr<Tire> VehicleFactory::create_tire(double _inertia, double _loadedRadius, double _cornerStiffness,
    double _slipStiffness)
{
  return std::make_shared<Tire>(_inertia,_loadedRadius,_cornerStiffness,_slipStiffness);
}

std::shared_ptr<VehicleSetup> VehicleFactory::create_vehicle_setup(std::shared_ptr<Chassis>& _chassis,
    std::shared_ptr<Aerodynamics>& _aero, std::shared_ptr<Tire>& _tire1, std::shared_ptr<Tire>& _tire2,
    std::shared_ptr<Tire>& _tire3, std::shared_ptr<Tire>& _tire4,
    std::shared_ptr<Brake>& _brakes,
    std::shared_ptr<Engine>& _engine)
{
  return std::make_shared<VehicleSetup>(_chassis,_aero,_engine,_brakes,_tire1,_tire2,_tire3,_tire4);
}

std::shared_ptr<Brake> VehicleFactory::create_brake(double bias, double maximum_torque)
{
  return std::make_shared<Brake>(bias,maximum_torque);
}

std::shared_ptr<Engine> VehicleFactory::create_engine(double maximum_power, double gear_ratio, double power_drag)
{
  return std::make_shared<Engine>(maximum_power, gear_ratio, power_drag);
}
