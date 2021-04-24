#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in,
            Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
  
  void UpdateCommon(const Eigen::VectorXd &y);

  void UpdateStateTransition(double dt);

  void UpdateNoiseCovarianceMatrix(double dt, double noise_x, double noise_y);

  void UpdateMesaurmentParameters(Eigen::MatrixXd const& H_in, Eigen::MatrixXd const& R_in);

  void UpdateStateVector(Eigen::VectorXd const& x_in);

  // state vector
  Eigen::VectorXd x_ = Eigen::VectorXd::Zero(4);

  // state covariance matrix
  Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(4,4);

  // state transition matrix
  Eigen::MatrixXd F_ = Eigen::MatrixXd::Identity(4,4);

  // process covariance matrix
  Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(4,4);

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif  // KALMAN_FILTER_H_
