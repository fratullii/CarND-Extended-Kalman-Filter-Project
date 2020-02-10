#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
                        Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in){

  // Initialize by reference
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  state_size_ = (unsigned int) x_.size();

}

void KalmanFilter::Predict() {

  // state prediction
  x_ = F_ * x_;

  // Covariance prediction
  P_ = F_ * P_ * F_.transpose() + Q_;

  return;
}

void KalmanFilter::Update(const VectorXd &z) {

  // compute residual
  y_ = z - H_ * x_;

  // compute residual covariance
  S_ = H_ * P_ * Ht_ + R_;

  // compute gain
  K_ = P_ * Ht_ * S_.inverse();

  // update state
  x_ = x_ + (K_ * y_);

  // update covariance
  P_ = (MatrixXd::Identity(state_size_,state_size_) - K_ * H_) * P_;

  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z,
                             VectorXd (*nonlinH)(const VectorXd&)){

  // compute residual with non linear function
  y_ = z - nonlinH(x_);

  // Normalize angle
  y_ = Tools::NormalizeAngle(y_);

  // compute residual covariance
  S_ = H_ * P_ * Ht_ + R_;

  // compute gain
  K_ = P_ * Ht_ * S_.inverse();

  // update state
  x_ = x_ + (K_ * y_);

  // update covariance
  P_ = (MatrixXd::Identity(state_size_, state_size_) - K_ * H_) * P_;

  return;
}


void KalmanFilter::set_R(const MatrixXd &R){
  R_ = R;
}
void KalmanFilter::set_H(const MatrixXd &H){
  H_ = H;
  Ht_ = H_.transpose();
}
void KalmanFilter::set_F(const MatrixXd &F){
  F_ = F;
}
void KalmanFilter::set_Q(const MatrixXd &Q){
  Q_ = Q;
}


MatrixXd KalmanFilter::get_R(){
  return R_;
}
MatrixXd KalmanFilter::get_H(){
  return H_;
}
MatrixXd KalmanFilter::get_F(){
  return F_;
}
MatrixXd KalmanFilter::get_Q(){
  return Q_;
}

VectorXd KalmanFilter::x_state(){
  return x_;
}
MatrixXd KalmanFilter::P_covariance(){
  return P_;
}