//
// Created by PedroCalorio on 8/30/2022.
//

#ifndef INC_4WM_TIRE_H
#define INC_4WM_TIRE_H

class Tire {

private:
  
  double _inertia{};
  double _loadedRadius{};
  double _cornerStiffness{};
  double _slipStiffness{};
  
  // in the future put a member variable that would be storing all the coefficients values

public:
  Tire(double _inertia, double _loadedRadius, double _cornerStiffness, double _slipStiffness);
  
  Tire() {  }
  
  void set_inertia_(double _inertia);
  
  void set_corner_stiffness_(double _cornerStiffness);
  
  void set_slip_stiffness_(double _slipStiffness);
  
  [[nodiscard]] double get_inertia_() const;
  
  [[nodiscard]] double get_corner_stiffness_() const;
  
  [[nodiscard]] double get_slip_stiffness_() const;
  
  [[nodiscard]] double get_loaded_radius_() const;
  
  [[nodiscard]] double get_alpha(double vsy, double vsx) const;
  
  [[nodiscard]] double get_kappa(double vsx, double omega) const;
  
  [[nodiscard]] double get_lateral_force(double alpha) const;
  
  [[nodiscard]] double get_longitudinal_force(double kappa) const;
  
  void get_tire_forces(double alpha,double kappa,
                       double& fx, double& fy) const;
  
};

#endif //INC_4WM_TIRE_H
