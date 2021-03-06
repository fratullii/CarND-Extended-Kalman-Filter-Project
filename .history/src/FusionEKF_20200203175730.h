#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

  /**
   * State vector getter
   */
  Eigen::VectorXd get_x();

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // Time step
  long long time_step_;

  // Last sensor used
  MeasurementPackage::SensorType last_sensor_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  // Matrices
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd Q_;

  // Process noise variances
  long long noise_ax_;
  long long noise_ay_;

  // For initialization
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
};

#endif // FusionEKF_H_
