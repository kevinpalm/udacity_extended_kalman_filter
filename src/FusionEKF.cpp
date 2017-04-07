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

  //observed state vector
  z_laser_ = VectorXd(4);
  z_laser_ << 0, 0, 0, 0,;
  z_radar_ = VectorXd(3);
  z_laser_ << 0, 0, 0;
  
  //save pi
  float pi = 3.1415926535897932;
  
  //save process noise sigma squared
  float x_noise = 9.0;
  float y_noise = 9.0;

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
    float t_ = measurement_pack.timestamp_;
    float delta_t;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Calculate x position
      ekf_.x_(0) = (cos(measurement_pack.phi)%(2*pi))*measurement_pack.ro_;
      
      // Calculate y position
      ekf_.x_(1) = (sin(measurement_pack.phi)%(2*pi))*measurement_pack.ro_;
      
      // Initialize filter
      ekf_.Init(ekf_.x_, P_, F_, H_, R_radar_, Q_);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      // Initialize filter
      ekf_.Init(ekf_.x_, P_, F_, H_, R_lidar_, Q_);
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   
   //Update the state transition matrix F according to the new elapsed time
   delta_t_ = measurement_pack.timestamp_ - t_;
   ekf_.F_(0, 2) = delta_t;
   ekf_.F_(1, 3) = delta_t;
   t_ = measurement_pack.timestamp_;
   
   //Update the process noise covariance matrix.
   ekf_.Q_(0, 0) = pow(delta_t, 4.0)/4.0*x_noise;
   ekf_.Q_(0, 2) = pow(delta_t, 3.0)/2*x_noise;
   ekf_.Q_(1, 1) = pow(delta_t, 4.0)/4.0*y_noise;
   ekf_.Q_(1, 3) = pow(delta_t, 3.0)/2*y_noise;
   ekf_.Q_(2, 0) = pow(delta_t, 3.0)/2*x_noise;
   ekf_.Q_(2, 2) = pow(delta_t, 2.0)*x_noise;
   ekf_.Q_(3, 1) = pow(delta_t, 3.0)/2*x_noise;
   ekf_.Q_(3, 3) = pow(delta_t, 2.0)*x_noise;
   

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
    
    //update the observed state
    z_radar_(0) = measurement_pack.ro;
    z_radar_(1) = measurement_pack.phi;
    z_radar_(2) = measurement_pack_ro_dot;
    
    //update the covariance matrix
    ekf_.R_ = R_radar_;
    
    //calculate jacovian and ensure H_
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    
    //run the update
    ekf_.UpdateEKF(z_);
    
  } else {
	  
    // Laser updates
    
    //update the observed state
    z_laser_(0) = measurement_pack.x_;
    z_laser_(1) = measurement_pack.y_;
    z_laser_(2) = (z_(0)-ekf_.x_(0))/(delta_t);
    z_laser_(3) = (z_(1)-ekf_.x_(1))/(delta_t);
    
    //update the covariance matrix
    ekf_.R_ = R_laser_;
    	
    // Ensure H_
    ekf_.H_ = H_laser_
    
    // Run the update
    ekf_.Update(z_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
