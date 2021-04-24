#include "kalman_filter.h"
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;


  // measurement covariance
  R_ = MatrixXd(2, 2);
  R_ << 0.0225, 0,
        0, 0.0225;

  // measurement matrix
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

}

KalmanFilter::~KalmanFilter() = default;

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
  // Done: predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Update the state by using Kalman Filter equations
  auto z_pred = H_ * x_;
  auto y = z - z_pred;
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Update the state by using Extended Kalman Filter equations
  auto z_pred = Tools::ConvertCartesianToPolar(x_);
  VectorXd y = z - z_pred;
  // normalize angle
  y(1) = atan2(sin(y(1)), cos(y(1)));
  UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd &y){  
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

void KalmanFilter::UpdateStateTransition(double dt){
  F_(0,2) = F_(1,3) = dt;
}

void KalmanFilter::UpdateNoiseCovarianceMatrix(double dt, double noise_x, double noise_y) {
  auto c1 = dt * dt;
  auto c2 = dt * c1 / 2;
  auto c3 = c1 * c1/ 4;

  Q_(0,0) = c3 * noise_x;
  Q_(0,1) = 0;
  Q_(0,2) = c2 * noise_x;
  Q_(0,3) = 0;

  Q_(1,0) = 0;
  Q_(1,1) = c3 * noise_y;
  Q_(1,2) = 0;
  Q_(1,3) = c2 * noise_y;

  Q_(2,0) = c2 * noise_x;
  Q_(2,1) = 0;
  Q_(2,2) = c1 * noise_x;
  Q_(2,3) = 0;
  
  Q_(3,0) = 0;
  Q_(3,1) = c2 * noise_y;
  Q_(3,2) = 0;
  Q_(3,3) = c1 * noise_y;
}

void KalmanFilter::UpdateMesaurmentParameters(Eigen::MatrixXd const& H_in, Eigen::MatrixXd const& R_in){
  H_ = H_in;
  R_ = R_in;
}

void KalmanFilter::UpdateStateVector(Eigen::VectorXd const& x_in){
  x_ = x_in;
}