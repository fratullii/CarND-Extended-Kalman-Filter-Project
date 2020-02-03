#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
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

void set_F(const Eigen::MatrixXd &F){
  F_ = F;
}

void set_H(const Eigen::MatrixXd &H){
  H_ = H;
}
void set_Q(const Eigen::MatrixXd &Q);
void set_R(const Eigen::MatrixXd &R);

/**
 * Getters
 */
void get_F(const Eigen::MatrixXd &F);
void get_H(const Eigen::MatrixXd &H);
void get_Q(const Eigen::MatrixXd &Q);
void get_R(const Eigen::MatrixXd &R);
