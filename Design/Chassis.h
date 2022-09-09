//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_CHASSIS_H
#define INC_4WM_CHASSIS_H

#include "Eigen/Dense"

class Chassis {

private:
  
  double _mass;
  double _yawInertia;
  double _weightDistribution;
  double _wheelbase;
  double _frontTrack;
  double _rearTrack;
  
public:
  
  Chassis(double _mass, double _yawInertia, double _weightDistribution, double _wheelbase, double _frontTrack,
      double _rearTrack);
  Chassis();
  
  
  
  [[nodiscard]] double get_mass_() const;
  
  [[nodiscard]] double get_yaw_inertia_() const;
  
  [[nodiscard]] double get_weight_distribution_() const;
  
  [[nodiscard]] double get_wheelbase_() const;
  
  [[nodiscard]] double get_front_track_() const;
  
  [[nodiscard]] double get_rear_track_() const;
  
  void set_mass_(double _mass);
  
  void set_yaw_inertia_(double _yawInertia);
  
  void set_weight_distribution_(double _weightDistribution);
  
  void set_wheelbase_(double _wheelbase);
  
  void set_front_track_(double _frontTrack);
  
  void set_rear_track_(double _rearTrack);
  
};

#endif //INC_4WM_CHASSIS_H
