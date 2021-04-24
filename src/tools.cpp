#include "tools.h"

#include <algorithm>
#include <iostream>
#include <numeric>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse = VectorXd(4);

  if (estimations.size() == 0 || ground_truth.size() == 0) {
    std::cout << "Cannot calculate RMSE for empty vectors";
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

std::pair<bool, Eigen::MatrixXd> Tools::CalculateJacobian(
    const Eigen::VectorXd &x_state) {
  // Done: Calculate a Jacobian here.
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  auto c1 = px * px + py * py;

  if (c1 < 0.0001) {
    std::make_pair(true, MatrixXd(3, 4));
  }

  auto c2 = std::sqrt(c1);
  auto c3 = c1 * c2;

  auto c4 = vx * py - vy * px;

  MatrixXd Hj = MatrixXd(3, 4);
  Hj(0, 2) = Hj(0, 3) = Hj(1, 2) = Hj(1, 3) = 0;

  Hj(0, 0) = px / c2;
  Hj(0, 1) = py / c2;

  Hj(1, 0) = -py / c1;
  Hj(1, 1) = px / c1;

  Hj(2, 0) = py * c4 / c3;
  Hj(2, 1) = py * (c4 * -1) / c3;
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1);

  return std::make_pair(true, Hj);
}

Eigen::VectorXd Tools::ConvertPolarToCartesian(VectorXd const &mesaurment) {
  auto x = mesaurment(0) * cos(mesaurment(1));
  auto y = mesaurment(0) * sin(mesaurment(1));
  auto cartesian = Eigen::VectorXd(4);
  cartesian << x, y, 0, 0;
  return cartesian;
}
Eigen::VectorXd Tools::ConvertCartesianToPolar(VectorXd const &mesaurment) {
  auto polar = VectorXd(3);
  polar(0) = std::sqrt(mesaurment(0) * mesaurment(0) + mesaurment(1) * mesaurment(1)); // range
  polar(1) = std::atan2(mesaurment(1), mesaurment(0)); // bearing
  
  polar(2) = std::abs(polar(0)) >= 0.0001 ? (mesaurment(0)*mesaurment(2) + mesaurment(1)*mesaurment(3)) / polar(0) : 0; // range rate
  return polar;
}

