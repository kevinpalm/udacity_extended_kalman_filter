#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {

	// Predict the state
	x_ = F_ * x_;

	// Predict the error
	Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
	
	// Compute the gain
	z_pred = H_ * x_;
	y = z - z_pred;
	Ht = H_.transpose();
	S = H_ * P_ * Ht + R_;
	Si = S.inverse();
	PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// Update the state
	x_ = x_ + (K * y);
	
	// Update the error
	x_size = x_.size();
	I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Convert predicted state to polar (h(x'))
    z_pred = VectorXd(3);
    z_pred(0) = sqrt(pow(x_(0), 2.0) + pow(x_(1), 2.0));
    z_pred(1) = atan(x_(1)/x_(0));
    z_pred(2) = (x_(0)*x_(2) + x_(1)*x_(3))/z_pred(0);
  
 	// Compute the gain
	y = z - z_pred;
	Ht = H_.transpose();
	S = H_ * P_ * Ht + R_;
	Si = S.inverse();
	PHt = P_ * Ht;
	K = PHt * Si;

	// Update the state
	x_ = x_ + (K * y);
	
	// Update the error
	x_size = x_.size();
	I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
 
 
}
