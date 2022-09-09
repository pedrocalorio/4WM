//
// Created by PedroCalorio on 9/8/2022.
//

#ifndef INC_4WM_CUSTOMSTEER_H
#define INC_4WM_CUSTOMSTEER_H

#include "SteeringInput.h"

class CustomSteer : public SteeringInput {

protected:
//  Eigen::VectorXd time_data, steering_data;
  
public:
  CustomSteer(const Eigen::VectorXd& time_data, const Eigen::VectorXd& steering_data);
  
  virtual ~CustomSteer();
  
  void generate_steering_profile(double simTime) override;
  
};

#endif //INC_4WM_CUSTOMSTEER_H
