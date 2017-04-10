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
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
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

  //Process covariance matrix = "process noise"
  Q_ = MatrixXd(4, 4);
  Q_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
  
  //save pi
  pi = 3.1415926535897932;
  
  //save process noise sigma squared
  x_noise = 9.0;
  y_noise = 9.0;
  
  // placeholders for z
  z_laser_ = VectorXd(2);
  z_radar_ = VectorXd(3);

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

    // first measurement... we'll assume velocity is 0 for start
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;
    
    //save timestamp
    current_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Calculate x position
      ekf_.x_(0) = cos(fmod(measurement_pack.raw_measurements_(1), 2*pi))*measurement_pack.raw_measurements_(0);
      
      // Calculate y position
      ekf_.x_(1) = sin(fmod(measurement_pack.raw_measurements_(1), 2*pi))*measurement_pack.raw_measurements_(0);
      
      // Initialize filter
      ekf_.Init(ekf_.x_, P_, F_, Hj_, R_radar_, Q_);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		
	  // Fill start positions
	  ekf_.x_(0) = measurement_pack.raw_measurements_(0);
	  ekf_.x_(1) = measurement_pack.raw_measurements_(1);

      // Initialize filter
      ekf_.Init(ekf_.x_, P_, F_, H_laser_, R_laser_, Q_);
      
    }
    
    // Record the state
    cout << "Starting x_" << endl;
    cout << ekf_.x_ << endl;
    cout << "Starting P_" << endl;
    cout << ekf_.P_ << endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   
   //Update the state transition matrix F according to the new elapsed time
   previous_timestamp_ = current_timestamp_;
   current_timestamp_ = measurement_pack.timestamp_;
   delta_t = current_timestamp_ - previous_timestamp_;
   cout << "Previous Timestamp:" << endl;
   cout << previous_timestamp_ << endl;
   cout << "Current timestamp:" << endl;
   cout << current_timestamp_ << endl;
   cout << "Delta T:" << endl;
   cout << delta_t << endl;
   
   // Only predict if delta_t is not 0 (no simultaneous readings)
   if (delta_t >= 0) {
	   
	   // Update the F matrix
	   ekf_.F_(0, 2) = delta_t;
	   ekf_.F_(1, 3) = delta_t;
	   
	   cout << "F Matrix:" << endl;
	   cout << ekf_.F_ << endl;
	   
	   //Update the process noise covariance matrix.
	   ekf_.Q_(0, 0) = pow(delta_t, 4.0)/4.0*x_noise;
	   ekf_.Q_(0, 2) = pow(delta_t, 3.0)/2*x_noise;
	   ekf_.Q_(1, 1) = pow(delta_t, 4.0)/4.0*y_noise;
	   ekf_.Q_(1, 3) = pow(delta_t, 3.0)/2*y_noise;
	   ekf_.Q_(2, 0) = pow(delta_t, 3.0)/2*x_noise;
	   ekf_.Q_(2, 2) = pow(delta_t, 2.0)*x_noise;
	   ekf_.Q_(3, 1) = pow(delta_t, 3.0)/2*x_noise;
	   ekf_.Q_(3, 3) = pow(delta_t, 2.0)*x_noise;
	   
	   cout << "Q Matrix:" << endl;
	   cout << ekf_.Q_ << endl;
	   
	   // Record the old state
	   ekf_.Predict();
	}
   
   // print the output
   cout << "x_predicted = " << ekf_.x_ << endl;
   cout << "P_predicted = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  
    // Radar updates
    
    //pass along observations
    z_radar_(0) = measurement_pack.raw_measurements_(0);
    z_radar_(1) = measurement_pack.raw_measurements_(1);
    z_radar_(2) = measurement_pack.raw_measurements_(2);
    
    //update the covariance matrix
    ekf_.R_ = R_radar_;
    
    //calculate jacovian and ensure H_
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    cout << Hj_ << endl;
    
    //run the update, if the jacovian is good
    if (tools.usej) {
		ekf_.UpdateEKF(z_radar_);
    }
    
  } else {
	  
    // Laser updates
    cout << "Raw laser Measurement:" << endl;
    cout << measurement_pack.raw_measurements_ << endl;
    
    //Pass along observations
    z_laser_(0) = measurement_pack.raw_measurements_(0);
    z_laser_(1) = measurement_pack.raw_measurements_(1);
    
    //update the covariance matrix
    ekf_.R_ = R_laser_;
    	
    // Ensure H_
    ekf_.H_ = H_laser_;
    
    // Run the update
    ekf_.Update(z_laser_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
