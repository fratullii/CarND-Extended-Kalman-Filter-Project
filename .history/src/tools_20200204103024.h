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
  void CalculateProcNoiseCov(Eigen::MatrixXd &Q, const long long &dt,
                             const long long &sigma_ax, const long long &sigma_ay);

  /**
   * A helper method to computer the state transition matrix
   */
  void CalculateStateTrans(Eigen::MatrixXd &F, const long long &dt);

  /**
   * A helper method to compute the non-linear transfer function state-measurement space
   */
  Eigen::VectorXd NonLinearH(Eigen::VectorXd &x_);
  
  /**
   * A helper method to calculate Jacobians.
   */
  void CalculateJacobian(Eigen::MatrixXd &Hj, const Eigen::VectorXd &x_state);

  /**
   * A helper method to pass from polar to cartesian coordinates.
   */
  void FromPolar2Cartesian(Eigen::VectorXd &x, const Eigen::VectorXd &raw_meas);111

};

#endif  // TOOLS_H_
