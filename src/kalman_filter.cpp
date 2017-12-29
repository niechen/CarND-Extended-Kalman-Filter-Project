#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
//  cout << "Update: " << endl;
  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

//  cout << "y = " << y << endl;
//  cout << "S = " << S << endl;
//  cout << "K = " << K << endl;

  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = VectorXd(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double rho = sqrt(px*px + py*py);
  double angle = atan2(py, px);
  double rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy)/rho;
  }


  z_pred << rho, angle, rho_dot;

  MatrixXd y = z - z_pred;

  // normalize angles
  while (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  };
  while (y(1) < -M_PI) {
    y(1) += 2*M_PI;
  };

  cout << "z = " << z << endl;
  cout << "z_pred = " << z_pred << endl;
  cout << "y = " << y << endl;



  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}
