#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

// Constructor
Tools::Tools() {}

// Destructor
Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size
   if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
      std::cout << "Invalid estimation or ground_truth data" << std::endl;
      return rmse;
   }

   // accumulate squared residuals
   for (unsigned int i=0; i < estimations.size(); ++i) {

      VectorXd residual = estimations[i] - ground_truth[i];

      // coefficient-wise multiplication
      residual = residual.array()*residual.array();
      rmse += residual;
   }

   // calculate the mean
   rmse = rmse/estimations.size();

   // calculate the squared root
   rmse = rmse.array().sqrt();

   // return the result
   return rmse;
}

void Tools::CalculateProcNoiseCov(MatrixXd &Q, const long long &dt,
                           const long long &sigma_ax, const long long &sigma_ay){

   // dt terms
   long long dt2 = dt * dt;
   long long dt3 = dt * dt2;
   long long dt4 = dt * dt3;

   // elements on the diagonal
   Q(0,0) = dt4 / 4 * sigma_ax * sigma_ax;
   Q(1,1) = dt4 / 4 * sigma_ay * sigma_ay;
   Q(2,2) = dt2 * sigma_ax * sigma_ax;
   Q(3,3) = dt2 * sigma_ay * sigma_ay;

   // elements off the diagonal
   Q(0,2) = dt3 / 2 * sigma_ax * sigma_ax;
   Q(1,3) = dt3 / 2 * sigma_ay * sigma_ay;
   Q(2,0) = Q(0,2);
   Q(3,1) = Q(1,3);

   return;
}

void Tools::CalculateStateTrans(MatrixXd &F, const long long &dt){

   F(0,2) = dt;
   F(1,3) = dt;

   return;
}

VectorXd Tools::NonLinearH(const VectorXd &x_state){

   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   float c1 = px*px+py*py;
   float c2 = sqrt(c1);

   // initialize vector in measurement space
   VectorXd hx(3);
   hx(0) = c2;
   hx(1) = atan2(py, px);
   hx(2) = (px*vx + py*vy) / c2;

   return hx;
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

void Tools::FromPolar2Cartesian(VectorXd &x, const VectorXd &raw_meas){

   x << raw_meas[0] * cos(raw_meas[1]),
        raw_meas[0] * sin(raw_meas[1]),
        0,
        0;

   return;
}


