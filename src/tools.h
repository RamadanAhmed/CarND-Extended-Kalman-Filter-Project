#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>

#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  static Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd> &estimations,
      const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static std::pair<bool, Eigen::MatrixXd> CalculateJacobian(const Eigen::VectorXd &x_state);

  static Eigen::VectorXd ConvertPolarToCartesian(Eigen::VectorXd const &mesaurment);

  static Eigen::VectorXd ConvertCartesianToPolar(Eigen::VectorXd const &mesaurment);
};

#endif  // TOOLS_H_
