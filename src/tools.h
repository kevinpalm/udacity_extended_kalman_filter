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
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  // check if the update should use the jacobian
  bool usej;

private:

  // rmse
  Eigen::VectorXd rmse;

  // jacobian matrix
  Eigen::MatrixXd Hj;

  //recover state parameters
  float px;
  float py;
  float vx;
  float vy;

  //pre-compute a set of terms to avoid repeated calculation
  float c1;
  float c2;
  float c3;

};

#endif /* TOOLS_H_ */
