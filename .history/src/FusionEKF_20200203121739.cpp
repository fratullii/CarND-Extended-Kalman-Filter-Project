#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {

  // Variable for initialization
  is_initialized_ = false;
  previous_timestamp_ = 0;

  /**
   * Initialize variables for FusionEKF that never change
   */

  // Measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar measurement matrix NOW WRONG!!!
  Hj_ = MatrixXd(3, 4);
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

  // Process noise variances
  noise_ax_ = 9.;
  noise_ay_ = 9.;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

    // Define state vector/ TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

    cout << "EKF: " << end/ TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

    x_in_ = VectorXd(4);

    // Initialize initiali/ TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

    F_in_ = MatrixXd(4,4);/ TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

    P_in_ = MatrixXd(4,4);
    Q_in_ = MatrixXd(4,4);

    // State transition matrix F
    F_in_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // state covariance matrix P
    P_in_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    // Process noise covariance matrix Q
    time_step_ = measurement_pack.timestamp_ - previous_timestamp_;
    Q_in_ = tools.CalculateProcNoiseCov(time_step_, noise_ax_, noise_ay_);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      Q_in_ = tools.FromCartesian2Polar(measurement_pack.raw_measurements_)
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
