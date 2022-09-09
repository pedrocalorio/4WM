//
// Created by PedroCalorio on 9/8/2022.
//

#ifndef INC_4WM_ENGINE_H
#define INC_4WM_ENGINE_H

class Engine {

protected:
  
  double maximum_power;
  double gear_ratio;
  double power_drag;
  
public:
  
  Engine(double maximum_power, double gear_ratio, double power_drag);
  
  double get_maximum_power_() const;
  
  void set_maximum_power_(double maximum_power);
  
  double get_gear_ratio_() const;
  
  void set_gear_ratio_(double gear_ratio);
  
  double get_power_drag_() const;
  
  void set_power_drag_(double power_drag);
  
};

#endif //INC_4WM_ENGINE_H
