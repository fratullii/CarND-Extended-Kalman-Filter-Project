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
   * Initialize variables for FusionEKF
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

  // Radar measurement matrix
  Hj_ = MatrixXd(3, 4);
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

  // State transition matrix F
  F_ = MatrixXd::Identity(4, 4);

  // Process noise variances
  noise_ax_ = 9.;
  noise_ay_ = 9.;

  // Process noise matrix
  Q_ = MatrixXd::Zero(4,4);

  // State vector x
  x_in_ = VectorXd(4);
  x_in_ << 1,1,1,1;

  // state covariance matrix P
  P_in_ = MatrixXd(4,4);
  P_in_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

}

/**
 * Destructor
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  // Time stamp filter
  time_step_ = measurement_pack.timestamp_ - previous_timestamp_;

  /**
   * Initialization
   */
  if (!is_initialized_) {

    // Process noise covariance matrix Q
    tools.CalculateProcNoiseCov(Q_, time_step_, noise_ax_, noise_ay_);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      tools.FromCartesian2Polar(x_in_, measurement_pack.raw_measurements_);
      ekf_.Init(x_in_, P_in_);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      x_in_ = VectorXd(4);
      x_in_ << measurement_pack.raw_measurements_[0],
              measurement_pack.raw_measurements_[1],
              0,
              0;

    }

    last_sensor_ = MeasurementPackage::NOSENSOR;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  tools.CalculateStateTrans(F_, time_step_);
  tools.CalculateProcNoiseCov(Q_, time_step_, noise_ax_,  noise_ay_);
  ekf_.set_F(F_);
  ekf_.set_Q(Q_);
  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    if (last_sensor_ == MeasurementPackage::LASER){
      ekf_.set_R(R_radar_);
    }
    tools.CalculateJacobian(Hj_, ekf_.get_x());
    ekf_.set_H(Hj_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
    // Laser updates

    if(last_sensor_ == MeasurementPackage::RADAR){
      ekf_.set_R(R_laser_);
      ekf_.set_H(H_laser_);
    }
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Update last_sensor value
  last_sensor_ = measurement_pack.sensor_type_;

  // print the output
  cout << "EKF: " << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
