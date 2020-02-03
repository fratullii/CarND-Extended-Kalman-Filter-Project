#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper to calculate the process noise covariance matrix Q
   */
  Eigen::MatrixXd CalculateProcNoiseCov(const long long &dt,
                                        const long long &sigma_ax,
                                        const long long &sigma_ay);

  /**
   * A helper method to computer the state transition matrix
   */
  Eigen::MatrixXd CalculateStateTrans(const long long &dt);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * A helper method to pass from cartesian to polar coordinates
   */
  Eigen::MatrixXd FromCartesian2Polar(Eigen::VectorXd raw_measurements);

};

#endif  // TOOLS_H_
