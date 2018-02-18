#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;


#define RADAR_POS_VECTOR_LENGTH 3 //position vector length for radar
#define LASER_POS_VECTOR_LENGTH 2 //position vector length for laser
// #define PI 3.141592653589793;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //apply non-linear formular
  double rho = std::pow(std::pow(x_(0),2) + std::pow(x_(1) ,2), 0.5);
  double phi = std::atan2(x_(1) , x_(0));
  double partial = x_(0) * x_(2) + x_(1) * x_(3);
  double rodot;
  rodot = rho != 0 ? partial/rho : 0;

  VectorXd z_pred = VectorXd(3); 
  z_pred << rho , phi , rodot;
  VectorXd y = z - z_pred;
  //adjust resulting phi' to -PI ~ PI
  y(1) = std::atan2( std::sin(y(1)) , std::cos(y(1)));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; 
}
