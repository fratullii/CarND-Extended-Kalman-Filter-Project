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

  // Process noise covariance matrix
  Q_ = MatrixXd::Zero(4,4);

  // State vector x
  x_ = VectorXd(4);
  x_ << 1,1,1,1;

  // State covariance matrix P
  P_ = MatrixXd(4,4);
  P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

}

/**
 * Destructor
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  // Update time-related variables
  time_step_ = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;

  /**
   * INITIALIZATION
   * Initialize the code with the first measurement
   */
  if (!is_initialized_) {

    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){

      tools.FromPolar2Cartesian(x_, measurement_pack.raw_measurements_);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){

      x_ << measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1],
            0,
            0;

    }

    ekf_.Init(x_, P_, F_, Q_);
    last_sensor_ = MeasurementPackage::NOSENSOR;
    is_initialized_ = true;
    return;
  }

  /**
   * PREDICTION
   */
  tools.CalculateStateTrans(F_, time_step_);
  tools.CalculateProcNoiseCov(Q_, time_step_, noise_ax_,  noise_ay_);
  // Actually, just setting the refernce with init should be enough to change the value
  // in the ekf_ object as well.
  // ekf_.set_F(F_);
  // ekf_.set_Q(Q_);
  ekf_.Predict();

  /**
   * UPDATE
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    if (last_sensor_ != MeasurementPackage::RADAR){
      ekf_.set_R(R_radar_);
      ekf_.set_H(Hj_);
    }
    tools.CalculateJacobian(Hj_, x_);
    // ekf_.set_H(Hj_); same reason as above: moved in the if statement, previous reference should be enough
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
    // Laser updates

    if(last_sensor_ != MeasurementPackage::LASER){
      ekf_.set_R(R_laser_);
      ekf_.set_H(H_laser_);
    }
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // Update last_sensor value
  last_sensor_ = measurement_pack.sensor_type_;

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

VectorXd get_x(){
  return x_;
}
