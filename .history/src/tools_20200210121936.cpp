#include "tools.h"
#include <iostream>

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

MatrixXd Tools::CalculateProcNoiseCov(const double &dt,
                           const double &sigma_ax, const double &sigma_ay){

   MatrixXd Q;
   std::cout << "timestep inside : " << dt << std::endl;
   Q = MatrixXd(4, 4);
   Q << 2, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 7, 0,
         0, 0, 0, 9;

   // dt and sigma terms
   double dt2 = dt * dt;
   double dt3 = dt * dt2;
   double dt4 = dt * dt3;

   // elements on the diagonal
   Q(0,0) = dt4 / 4 * sigma_ax;
   Q(1,1) = dt4 / 4 * sigma_ay;
   Q(2,2) = dt2 * sigma_ax;
   Q(3,3) = dt2 * sigma_ay;

   // elements off the diagonal
   Q(0,2) = dt3 / 2 * sigma_ax;
   Q(1,3) = dt3 / 2 * sigma_ay;
   Q(2,0) = Q(0,2);
   Q(3,1) = Q(1,3);

   return Q;
}

MatrixXd Tools::CalculateStateTrans(const double &dt){

   MatrixXd F = MatrixXd::Identity(4,4);

   F(0,2) = dt;
   F(1,3) = dt;

   return F;
}

VectorXd Tools::NonLinearH(const VectorXd &x_state){

   // initialize vector in measurement space
   VectorXd hx(3);
   hx(0) = sqrt(x_state(0)*x_state(0) + x_state(1)*x_state(1));
   hx(1) = atan2(x_state(1), x_state(0));
   hx(2) = (x_state(0)*x_state(2) + x_state(1)*x_state(3)) / hx(0);

   return hx;
}

 MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {

  MatrixXd Hj = MatrixXd(3,4);

  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  double c1 = px*px + py*py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;

  // check division by zero
  if (fabs(c1) < 0.0001) {
     std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return Hj;
}

VectorXd Tools::FromPolar2Cartesian(const VectorXd &raw_meas){

   VectorXd x = VectorXd(4);
   x << raw_meas[0] * cos(raw_meas[1]),
        raw_meas[0] * sin(raw_meas[1]),
        raw_meas[2] * cos(raw_meas[1]),
        raw_meas[2] * sin(raw_meas[1]);

   return x;
}

VectorXd Tools::NormalizeAngle(const VectorXd &y){

   VectorXd ynorm = y;

   while (ynorm(1)>M_PI) {
      ynorm(1) -= 2 * M_PI;
  }
  while (ynorm(1)<-M_PI) {
      ynorm(1) += 2 * M_PI;
  }

  return ynorm;
}


