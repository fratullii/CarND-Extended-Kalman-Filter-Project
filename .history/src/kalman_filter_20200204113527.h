#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
            Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z,
                 Eigen::VectorXd (*nonlinH)(const int));

  /**
   * Setters
   */

  void set_R(const Eigen::MatrixXd &R);
  void set_H(const Eigen::MatrixXd &H);
  void set_F(const Eigen::MatrixXd &F);
  void set_Q(const Eigen::MatrixXd &Q);

  /**
   * Getters
   */
  Eigen::MatrixXd get_R();
  Eigen::MatrixXd get_H();
  Eigen::MatrixXd get_F();
  Eigen::MatrixXd get_Q();
  Eigen::VectorXd get_x();
  Eigen::MatrixXd get_P();

  private:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // residual vector
  Eigen::VectorXd y_;

  // S
  Eigen::MatrixXd S_;

  // gain matrix
  Eigen::MatrixXd K_;
};

#endif // KALMAN_FILTER_H_
