#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "math.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
        
  //state Transition matrix = "motion model for prediction step"
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
  
  //(initial) state covariance = "uncertanity"
  P_ = MatrixXd(4, 4);
  P_ << 10, 0, 0, 0,
		0, 10, 0, 0,
		0, 0, 10, 0,
		0, 0, 0, 10;
		
  //Measurement matrix = "measurement mode for update step"
  H_ = MatrixXd(4, 4);
  H_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

  //Process covariance matrix = "process noise"
  Q_ = MatrixXd(4, 4);
  Q_ << 9, 0, 0, 0,
		0, 9, 0, 0,
		0, 0, 9, 0,
		0, 0, 0, 9;
  
  //save pi
  float pi = 3.1415926535897932;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement... we'll assume velocity is 0 for start
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;
    
    // Create filter
    KalmanFilter filter;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Calculate x position
      ekf_.x_(0) = (cos(measurement_pack.phi)%pi)*measurement_pack.ro_;
      
      // Calculate y position
      ekf_.x_(1) = (sin(measurement_pack.phi)%pi)*measurement_pack.ro_;
      
      // Initialize filter
      filter.Init(ekf_.x_, P_, F_, H_, R_radar_, Q_);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      // Initialize filter
      filter.Init(ekf_.x_, P_, F_, H_, R_lidar_, Q_);
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  
    // Radar updates
    
  } else {
	  
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
