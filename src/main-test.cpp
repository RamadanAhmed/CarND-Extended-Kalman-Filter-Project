#include <cmath>
#include <fstream>
#include <iostream>

#include "FusionEKF.h"
#include "tools.h"

int main() {
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  FusionEKF fusionEKF;

  std::ifstream sensor_file(
      "/root/CarND-Extended-Kalman-Filter-Project/data/"
      "obj_pose-laser-radar-synthetic-input.txt");
  
  std::string line;
  
  while (std::getline(sensor_file, line)) {
    MeasurementPackage meas_package;

    std::istringstream ss(line);
    char type = '0';
    ss >> type;
    if (type == 'L') {
      double px = 0.0;
      double py = 0.0;
      long long timestamp = 0;

      ss >> px >> py >> timestamp;

      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      meas_package.raw_measurements_ << px, py;
      meas_package.timestamp_ = timestamp;
    } else {
      // continue;
      double ro = 0.0;
      double theta = 0.0;
      double ro_dot = 0.0;
      long long timestamp = 0;

      ss >> ro >> theta >> ro_dot >> timestamp;

      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = Eigen::VectorXd(3);
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      meas_package.timestamp_ = timestamp;
    }
    Eigen::VectorXd gt_values(4);
    ss >> gt_values(0) >> gt_values(1) >> gt_values(2) >> gt_values(3);
    ground_truth.push_back(gt_values);

    // Call ProcessMeasurement(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's
    //   state vector

    Eigen::VectorXd estimate(4);

    estimate(0) = fusionEKF.ekf_.x_(0);
    estimate(1) = fusionEKF.ekf_.x_(1);
    estimate(2) = fusionEKF.ekf_.x_(2);
    estimate(3) = fusionEKF.ekf_.x_(3);

    estimations.push_back(estimate);

    Eigen::VectorXd RMSE = Tools::CalculateRMSE(estimations, ground_truth);

    // std::cout << "Estimation : " << '\n';
    // std::cout << estimate << '\n';
    // std::cout << "GroundTruth : " << '\n';
    // std::cout << gt_values << '\n';
    std::cout << "CalculateRMSE : " << '\n';
    std::cout << RMSE << '\n';
    
  }
  // read Mesaurment

  // Compute Fusion
  // Compare with gt

  return 0;
}