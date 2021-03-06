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

void CalculateStateTrans(MatrixXd &F, const long long &dt){

   F(0,2) = dt;
   F(1,3) = dt;

   return;
}

 void Tools::CalculateJacobian(MatrixXd &Hj, const VectorXd &x_state) {

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
     std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return;
}

void FromPolar2Cartesian(VectorXd &x, const VectorXd &raw_meas){

   x << raw_meas[0] * cos(raw_meas[1]),
        raw_meas[0] * sin(raw_meas[1]),
        0,
        0;

   return;
}


