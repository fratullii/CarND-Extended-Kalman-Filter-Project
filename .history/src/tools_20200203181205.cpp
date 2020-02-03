#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

void CalculateProcNoiseCov(MatrixXd &Q, const long long &dt,
                           const long long &sigma_ax, const long long &sigma_ay){
   /**
    * Calculate Q
    */
}

void CalculateStateTrans(Eigen::MatrixXd &F, const long long &dt){
   // Calculate F
}

 void Tools::CalculateJacobian(MatrixXd &Hj, const VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}

void FromPolar2Cartesian(Eigen::VectorXd &x, const Eigen::VectorXd &raw_measurements){
   // calculate state x (only px and py) from rho and theta
}


