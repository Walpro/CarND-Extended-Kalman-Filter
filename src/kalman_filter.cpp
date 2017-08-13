#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */

	x_ = F_*x_; // to do

	P_ = F_*P_* F_.transpose() +Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd y;
	MatrixXd K,S, I;

	// identity 4 matrix
	I = MatrixXd(4,4);
	I << 1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1;
	// updating the state
	y = z - H_*x_;

	S = H_*P_*H_.transpose() + R_;

	K = P_*H_.transpose()*S.inverse();


	//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
	 TODO:
	 * update the state by using Extended Kalman Filter equations
	 */
	float rho, theta, rhod;
	float x1 = x_(0);
	float y1 = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	VectorXd y;
	VectorXd Hx = VectorXd(3);
	MatrixXd K, S, I;

	// identity matrix
	I = MatrixXd(4,4);
	I << 1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1;

	rho = sqrt(x1 * x1 + y1 * y1);
	theta = atan2(y1, x1);
	rhod = (x1 * vx + y1 * vy) / rho;

	Hx << rho,theta,rhod;

	// updating the state
	y = z - Hx;
	S = H_ * P_ * H_.transpose() + R_;
	K = P_ * H_.transpose() * S.inverse();

	//new state
	x_ = x_ + (K * y);
	P_ = (I -  K * H_)*P_;
}
