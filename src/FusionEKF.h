#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  
  // previous state for handling simultanious readings
  Eigen::VectorXd previous_state_;

  // previous timestamp
  long previous_timestamp_;
  long current_timestamp_;
  long delta_t;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  
  // Observed states
  Eigen::VectorXd z_laser_;
  Eigen::VectorXd z_radar_;

  // initialization and helper matrix
  Eigen::MatrixXd F_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;

  // helpers for polar/Cartesian conversions and process noise
  double pi;
  float x_noise;
  float y_noise;

  // time helpers
  
  
};

#endif /* FusionEKF_H_ */
