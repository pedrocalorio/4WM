//
// Created by PedroCalorio on 9/8/2022.
//

#include "CustomSteer.h"



CustomSteer::~CustomSteer()
{

}

void CustomSteer::generate_steering_profile(double simTime)
{

}

CustomSteer::CustomSteer(const Eigen::VectorXd& time_data, const Eigen::VectorXd& steering_data) {
  _timeData = time_data;
  _steeringData = steering_data;
}
