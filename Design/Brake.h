//
// Created by PedroCalorio on 9/8/2022.
//

#ifndef INC_4WM_BRAKE_H
#define INC_4WM_BRAKE_H

class Brake {

private:
  
  double brake_bias{};
  
  double maximum_torque{};

public:
  Brake(double brake_bias, double maximum_torque);
  
  double get_brake_bias_() const;
  
  void set_brake_bias_(double brake_bias);
  
  double get_maximum_torque_() const;
  
  void set_maximum_torque_(double maximum_torque);
  
};

#endif //INC_4WM_BRAKE_H
