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
   long long dt2 = dt * dt;
   long long dt3 = dt * dt2;
   long long dt4 = dt * dt3;

   Q(0,0) = dt4 / 4 * sigma_ax;
   Q(1,1) = dt4 / 4 * sigma_ay;
   Q(2,2) = dt2 * sigma_ax;
   Q(3,3) = dt2 * sigma_ay;

   Q(0,2) = dt3 / 3 * sigma_ax;
   Q(1,3) = dt3 / 3 * sigma_ax;
   Q(2,0) = Q(0,2);
   Q(3,1) = Q(1,3);

   return;
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
   // calculate state x (\only px and py) from rho and theta
}


