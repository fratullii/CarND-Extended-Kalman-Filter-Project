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
  Eigen::MatrixXd CalculateProcNoiseCov(const double &dt,
                                        const double &sigma_ax, const double &sigma_ay);

  /**
   * A helper method to computer the state transition matrix
   */
  Eigen::MatrixXd CalculateStateTrans(const double &dt);

  /**
   * A helper method to compute the non-linear transfer function state-measurement space
   */
  static Eigen::VectorXd NonLinearH(const Eigen::VectorXd &x_state);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

  /**
   * A helper method to pass from polar to cartesian coordinates.
   */
  Eigen::VectorXd FromPolar2Cartesian(const Eigen::VectorXd &raw_meas);

  static Eigen::VectorXd NormalizeAngle(const Eigen::VectorXd &y);

};

#endif  // TOOLS_H_
