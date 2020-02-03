#include "kalman_filter.h"

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
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}

/**
  * Setters
**/
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


/**
 * Getters
 */
MatrixXd KalmanFilter::get_F(){
  return F_;
}
MatrixXd KalmanFilter::get_H(){
  return H_;
}
MatrixXd KalmanFilter::get_Q(){
  return Q_;
}
MatrixXd KalmanFilter::get_R(){
  return R_;
}
VectorXd KalmanFilter::get_x(){
  return x_;
}

