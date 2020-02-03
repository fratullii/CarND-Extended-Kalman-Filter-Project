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

voide 

 void Tools::CalculateJacobian(MatrixXd &Hj, const VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}


