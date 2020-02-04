#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Tools::NonLinearH;

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

  // Initialize private hidden variables
  y_ = VectorXd(4);
  S_ = MatrixXd(4);
  K_ = MatrixXd(4);
}

void KalmanFilter::Predict() {

  // state prediction
  x_ = F_ * x_;

  // Covariance prediction
  P_ = F_.transpose() * P_ * F_ + Q_;

  return;
}

void KalmanFilter::Update(const VectorXd &z) {

  // compute residual
  y_ = z - 

  // compute residual covariance
  S_ = H_ * P_ * H_.transpose() + R_;

  // compute gain
  K_ = P_ * H_.transpose() * S_.inverse();

  // update state
  x_ = x_ + K_ * y_;

  // update covariance
  P_ = (MatrixXd::Identity(4,4) - K_ * H_) * P_;

  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // compute residual with non linear function
  y_ = z - tools::h_nonlinear;

  // compute residual covariance
  S_ = H_ * P_ * H_.transpose() + R_;

  // compute gain
  K_ = P_ * H_.transpose() * S_.inverse();

  // update state
  x_ = x_ + K_ * y_;

  // update covariance
  P_ = (MatrixXd::Identity(4,4) - K_ * H_) * P_;

  return;
}


void KalmanFilter::set_R(const MatrixXd &R){
  R_ = R;
}
void KalmanFilter::set_H(const MatrixXd &H){
  H_ = H;
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
VectorXd KalmanFilter::get_x(){
  return x_;
}
MatrixXd KalmanFilter::get_P(){
  return P_;
}


