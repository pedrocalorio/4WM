//
// Created by PedroCalorio on 8/30/2022.
//

#include <iostream>
#include "VehicleSetup.h"


VehicleSetup::VehicleSetup(const std::shared_ptr<Chassis>& _chassis, const std::shared_ptr<Aerodynamics>& _aerodynamics,
    const std::shared_ptr<Engine>& _engine, const std::shared_ptr<Brake>& _brake, const std::shared_ptr<Tire>& _tire1,
    const std::shared_ptr<Tire>& _tire2, const std::shared_ptr<Tire>& _tire3, const std::shared_ptr<Tire>& _tire4)
    :_chassis(_chassis), _aerodynamics(_aerodynamics), _engine(_engine), _brake(_brake), _tire1(_tire1), _tire2(_tire2),
     _tire3(_tire3), _tire4(_tire4) { }


Eigen::MatrixXd VehicleSetup::get_mass_matrix()
{
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(7,7);
  
  M(0,0) = this->_chassis->get_mass_();
  M(1,1) = this->_chassis->get_mass_();
  M(2,2) = this->_chassis->get_yaw_inertia_();
  M(3,3) = this->_tire1->get_inertia_();
  M(4,4) = this->_tire2->get_inertia_();
  M(5,5) = this->_tire3->get_inertia_();
  M(6,6) = this->_tire4->get_inertia_();
  
  return M;
}

Eigen::VectorXd VehicleSetup::get_forces_and_moments_vector(const std::shared_ptr<SimulationInput>& input,
    const double time, const Eigen::VectorXd& q, const Eigen::VectorXd& qDot)
{
  // declaring the size of the forces and moments vector
  Eigen::VectorXd f(7);
  
  // states of the dynamical system
  auto vx = qDot(0);
  auto vy = qDot(1);
  auto r = qDot(2);
  auto omega1 = qDot(3);
  auto omega2 = qDot(4);
  auto omega3 = qDot(5);
  auto omega4 = qDot(6);
  
  // local variables of the vehicle parameters
  auto m = _chassis->get_mass_();
  auto a = _chassis->get_wheelbase_()*(1-this->_chassis->get_weight_distribution_());
  auto b = _chassis->get_wheelbase_()*this->_chassis->get_weight_distribution_();
  auto frontTrack = _chassis->get_front_track_();
  auto rearTrack = _chassis->get_rear_track_();
  double steering_ratio = 13.71;
  
  auto Rl1 = _tire1->get_loaded_radius_();
  auto Rl2 = _tire2->get_loaded_radius_();
  auto Rl3 = _tire3->get_loaded_radius_();
  auto Rl4 = _tire4->get_loaded_radius_();
  
//  auto Iw1 = _tire1->get_inertia_();
  
  // local variables for the simulation inputs
//  std::shared_ptr<SteeringInput> steering_input            = input->get_delta_();
//  std::shared_ptr<LongitudinalControlInput> tau_input      = input->get_longitudinal_control_input_();

  
  double delta = (input->delta->get_steering_value(time)/steering_ratio)/57.3; //radians
  double tauPos = input->longitudinal_control_input->get_positive_tau_value(time)/100;
  double tauNeg = -input->longitudinal_control_input->get_negative_tau_value(time)/132.9260;
  
  
  auto vsy1 = (cos(delta)*(r*a+vy) - sin(delta)*(r*frontTrack/2 + vx));
  auto vsy2 = (sin(delta)*(r*frontTrack/2-vx) + cos(delta)*(r*a + vy));
  auto vsy3 = vy - r*b;
  auto vsy4 = vy - r*b;
  
  auto vsx1 = ( cos(delta)*(r*frontTrack/2 + vx) + sin(delta)*(r*a + vy) );
  auto vsx2 = ( cos(delta)*(-r*frontTrack/2+vx) + sin(delta)*(r*a+vy));
  auto vsx3 = vx+r*rearTrack/2;
  auto vsx4 = vx-r*rearTrack/2;
  
  auto alpha1 = _tire1->get_alpha(vsy1,vsx1);
  auto alpha2 = _tire2->get_alpha(vsy2,vsx2);
  auto alpha3 = _tire3->get_alpha(vsy3,vsx3);
  auto alpha4 = _tire4->get_alpha(vsy4,vsx4);
  
  auto kappa1 = _tire1->get_kappa(vsx1, omega1);
  auto kappa2 = _tire2->get_kappa(vsx2, omega2);
  auto kappa3 = _tire3->get_kappa(vsx3, omega3);
  auto kappa4 = _tire4->get_kappa(vsx4, omega4);
  
  double fx1{}, fy1{}, fx2{}, fy2{}, fx3{}, fy3{}, fx4{}, fy4{};
  
  _tire1->get_tire_forces(alpha1,kappa1,fx1,fy1);
  _tire2->get_tire_forces(alpha2,kappa2,fx2,fy2);
  _tire3->get_tire_forces(alpha3,kappa3,fx3,fy3);
  _tire4->get_tire_forces(alpha4,kappa4,fx4,fy4);
  
  double dragForce{};
  dragForce = _aerodynamics->get_drag_force(vx);
  
  double Fx{},Fy{};
  
  Fx = (fx1+fx2)*cos(delta) - (fy1+fy2)*sin(delta) + (fx3+fx4) - dragForce;
  
  Fy = (fx1+fx2)*sin(delta) + (fy1+fy2)*cos(delta) + fy3 + fy4;
  
  double YMfromFy{}, YMfromFx{};
  YMfromFy = a*( (fx1+fx2)*sin(delta) + (fy1+fy2)*cos(delta) ) - b*(fy3+fy4);
  YMfromFx = frontTrack/2*(cos(delta)*fx1-sin(delta)*fy1) + frontTrack/2*(sin(delta)*fy2-cos(delta)*fx2) - rearTrack/2*fx4 + rearTrack/2*fx3;
  
  Eigen::Vector4d wheel_torques{};
  get_wheels_torque(tauPos,tauNeg,qDot,wheel_torques);
  
  double torque1 = wheel_torques(0); double torque2 = wheel_torques(1); double torque3 = wheel_torques(2); double torque4 = wheel_torques(3);
  
  f(0) = Fx + r*vy*m;
  f(1) = Fy - r*vx*m;
  f(2) = YMfromFy + YMfromFx;
  f(3) = torque1 - Rl1*fx1;
  f(4) = torque2 - Rl2*fx2;
  f(5) = torque3 - Rl3*fx3;
  f(6) = torque4 - Rl4*fx4;
  
  return -f;
  
  
}

Eigen::MatrixXd VehicleSetup::get_stiffness_matrix(const std::shared_ptr<SimulationInput>& input, const double time,
    const Eigen::VectorXd& q, const Eigen::VectorXd& qDot)
{
  // gets the number of generalized coordinates
  auto nDoF = q.size();
  
  // defines the value of epsilon for numerically calculating the gradients
  double eps = 1e-4;
  
  Eigen::MatrixXd hDoF = eps * Eigen::MatrixXd::Identity(nDoF,nDoF);
  
  Eigen::MatrixXd qRepMat = q.replicate(1,nDoF);
  
  Eigen::MatrixXd X_PLUS_H(7,7);
  Eigen::MatrixXd X_MINUS_H(7,7);

  for (int i = 0; i < nDoF; ++i) {
    for (int j = 0; j < nDoF; ++j) {
      X_PLUS_H(i,j) = qRepMat(i,j) + hDoF(i,j);
      X_MINUS_H(i,j) = qRepMat(i,j) - hDoF(i,j);
    }
  }
  
//  Eigen::MatrixXd X_PLUS_H = qRepMat.array() + hDoF.array();
//  Eigen::MatrixXd X_MINUS_H = qRepMat.array() - hDoF.array();
  
  Eigen::MatrixXd jacobian = Eigen::MatrixXd(nDoF,nDoF);
  
  for (int i = 0; i < nDoF; ++ i) {
    Eigen::VectorXd x_plus_vec = X_PLUS_H.col(i).adjoint();
    Eigen::VectorXd x_minus_vec = X_MINUS_H.col(i).adjoint();
    
    Eigen::VectorXd F_X_PLUS_H = this->get_forces_and_moments_vector(input,time,x_plus_vec,qDot);
    Eigen::VectorXd F_X_MINUS_H = this->get_forces_and_moments_vector(input,time,x_minus_vec,qDot);
    
    Eigen::VectorXd F_X = F_X_PLUS_H-F_X_MINUS_H;
    jacobian.col(i) = F_X;

  }
  
  Eigen::MatrixXd eps_vec = 2*eps*Eigen::MatrixXd::Ones(nDoF,1).adjoint();

  Eigen::MatrixXd div = eps_vec.replicate(nDoF,1);
  
  Eigen::MatrixXd K = jacobian.cwiseQuotient(div);
  
  return K;
}

Eigen::MatrixXd VehicleSetup::get_damping_matrix(const std::shared_ptr<SimulationInput>& input, const double time,
    const Eigen::VectorXd& q, const Eigen::VectorXd& qDot)
{
  // gets the number of generalized coordinates
  auto nDoF = qDot.size();
  
  // defines the value of epsilon for numerically calculating the gradients
  double eps = 1e-4;
  
  Eigen::MatrixXd hDoF = eps * Eigen::MatrixXd::Identity(nDoF,nDoF);
  
  Eigen::MatrixXd qDotRepMat = qDot.replicate(1,nDoF);
  
  Eigen::MatrixXd X_PLUS_H(7,7);
  Eigen::MatrixXd X_MINUS_H(7,7);

  for (int i = 0; i < nDoF; ++i) {
    for (int j = 0; j < nDoF; ++j) {
      X_PLUS_H(i,j) = qDotRepMat(i,j) + hDoF(i,j);
      X_MINUS_H(i,j) = qDotRepMat(i,j) - hDoF(i,j);
    }
  }
  
//  Eigen::MatrixXd X_PLUS_H = qDotRepMat.array() + hDoF.array();
//  Eigen::MatrixXd X_MINUS_H = qDotRepMat.array() - hDoF.array();
  
  Eigen::MatrixXd jacobian = Eigen::MatrixXd(nDoF,nDoF);
  
  for (int i = 0; i < nDoF; ++ i) {
    Eigen::VectorXd x_plus_vec = X_PLUS_H.col(i).adjoint();
    Eigen::VectorXd x_minus_vec = X_MINUS_H.col(i).adjoint();
    
    Eigen::VectorXd F_X_PLUS_H = this->get_forces_and_moments_vector(input,time,q,x_plus_vec);
    Eigen::VectorXd F_X_MINUS_H = this->get_forces_and_moments_vector(input,time,q,x_minus_vec);
    
    Eigen::VectorXd F_X = F_X_PLUS_H-F_X_MINUS_H;
    jacobian.col(i) = F_X;

  }
  
  Eigen::MatrixXd eps_vec = 2*eps*Eigen::MatrixXd::Ones(nDoF,1).adjoint();
  
  Eigen::MatrixXd div = eps_vec.replicate(nDoF,1);
  
  Eigen::MatrixXd D = jacobian.cwiseQuotient(div);
  
//  for (int j = 0; j < nDoF ; ++ j) {
//    for (int i = 0; i < nDoF; ++i) {
//      std::cout << D(j,i) << " ";
//    }
//    std::cout << "\n";
//  }
  
  return D;
}

void VehicleSetup::get_wheels_torque(const double& tau_pos, const double& tau_neg, const Eigen::VectorXd& states, Eigen::Vector4d& wheel_torques)
{
  
  auto omega3 = states(5);
  auto omega4 = states(6);
  
  auto brake_bias = _brake->get_brake_bias_();
  auto brake_maximum_torque = _brake->get_maximum_torque_();
  
  auto power_engine = _engine->get_maximum_power_();
  auto gear_ratio = _engine->get_gear_ratio_();
  auto power_drag = _engine->get_power_drag_();
  
  auto omega_engine = (omega3+omega4)/2 * gear_ratio;
  
  auto torque_engine_drag = 2*power_drag/(omega3+omega4);
  auto torque_engine = tau_pos*(power_engine/omega_engine);
  
  auto torque_front = brake_bias*brake_maximum_torque*tau_neg;
  auto torque_rear_brake = ((1-brake_bias)*brake_maximum_torque + torque_engine_drag )*tau_neg;
  
  wheel_torques(0) = torque_front/2;
  wheel_torques(1) = torque_front/2;
  wheel_torques(2) = (torque_engine*gear_ratio + torque_rear_brake)/2;
  wheel_torques(3) = (torque_engine*gear_ratio + torque_rear_brake)/2;

}
