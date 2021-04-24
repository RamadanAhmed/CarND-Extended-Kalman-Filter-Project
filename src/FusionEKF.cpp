#include "FusionEKF.h"

#include <iostream>

#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // Finish initializing the FusionEKF.
  // Set the process and measurement noises

  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  std::cout << "Current Mes : " <<measurement_pack.raw_measurements_ << std::endl;
  
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    auto x = VectorXd(4);
    x << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates
      x = tools.ConvertPolarToCartesian(measurement_pack.raw_measurements_);
      // Create the covariance matrix.
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x(0) = measurement_pack.raw_measurements_(0);
      x(1) = measurement_pack.raw_measurements_(1);
      x(2) = x(3) = 0;
      // Create the covariance matrix.
    }
    // Initialize the state ekf_.x_ with the first measurement.
    ekf_.UpdateStateVector(x);
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  previous_timestamp_ = measurement_pack.timestamp_;
  // Update the state transition matrix F according to the new elapsed
  // time(seconds)
  ekf_.UpdateStateTransition(dt);
  // Update the process noise covariance matrix.
  ekf_.UpdateNoiseCovarianceMatrix(dt, 9, 9);

  ekf_.Predict();

  /**
   * Update
   */
  //
  // Use the sensor type to perform the update step.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    auto result = tools.CalculateJacobian(ekf_.x_);

    if (result.first) {
      Hj_ = result.second;
    }

    ekf_.UpdateMesaurmentParameters(Hj_, R_radar_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.UpdateMesaurmentParameters(H_laser_, R_laser_);
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
