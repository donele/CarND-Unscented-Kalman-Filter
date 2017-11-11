#include "KFRadar.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
const double PI = 3.14159265;

KFRadar::KFRadar() {
	// measurement matrix
	H_ = MatrixXd::Zero(3, 4);
	H_trans_ = MatrixXd::Zero(4, 3);

	// measurement noise
	R_ = MatrixXd(3, 3);
	R_ << 0.09, 0, 			0,
		 		0, 		0.0009, 0,
				0, 		0, 			0.09;
}

KFRadar::~KFRadar() {}

KFState KFRadar::Update(const KFState& stateIn, float dt, const VectorXd &z) {
	KFState statePred = Predict(stateIn, dt);

	// Keep the value of theta in [-pi, pi]
	VectorXd y(3);
	y = z - tools.Cartesian2Polar(statePred.x);
	float theta = y[1];
	while(theta > PI)
		theta -= 2*PI;
	while(theta < -PI)
		theta += 2*PI;
	y[1] = theta;

	// Calculate jacobian
	H_ = tools.CalculateJacobian(statePred.x);
	H_trans_ = H_.transpose();

	// Calculate Kalman gain
	MatrixXd K = statePred.P * H_trans_ * (H_ * statePred.P * H_trans_ + R_).inverse();

	// Update state and covariance matrices
	KFState stateOut;
	stateOut.x = statePred.x + K * y;
	stateOut.P = statePred.P - K * H_ * statePred.P;
	return stateOut;
}
