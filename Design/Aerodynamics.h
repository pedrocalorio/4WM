//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_AERODYNAMICS_H
#define INC_4WM_AERODYNAMICS_H

#include <cmath>

class Aerodynamics {

private:
  
  double _liftCoef{};
  double _dragCoef{};
  double _frontArea{};
  double _airDensity{};
  double _aerobalance{};

public:
  Aerodynamics(double _liftCoef, double _dragCoef, double _frontArea, double _airDensity, double _aerobalance);
  
  Aerodynamics() {
  
  }
  
  void set_lift_coef_(double _liftCoef);
  
  void set_drag_coef_(double _dragCoef);
  
  void set_front_area_(double _frontArea);
  
  void set_air_density_(double _airDensity);
  
  void set_aerobalance_(double _aerobalance);
  
  double get_lift_coef_() const;
  
  double get_drag_coef_() const;
  
  double get_front_area_() const;
  
  double get_air_density_() const;
  
  double get_aerobalance_() const;
  
  double get_drag_force(const double speed) const;
  
};

#endif //INC_4WM_AERODYNAMICS_H
