#include "KFLaser.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KFLaser::KFLaser() {
	// Measurement matrix.
	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
		 		0, 1, 0, 0;

	H_trans_ = H_.transpose();

	// Measurement covariance matrix.
	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
		 		0, 			0.0225;
}

KFLaser::~KFLaser() {}

KFState KFLaser::Update(const KFState& stateIn, float dt, const VectorXd &z) {
	// Predict the state from the elapsed time.
	KFState statePred = Predict(stateIn, dt);

	// Calculate Kalman gain
	MatrixXd K = statePred.P * H_trans_ * (H_ * statePred.P * H_trans_ + R_).inverse();

	// Update the state from the measurement.
	KFState stateOut;
	stateOut.x = statePred.x + K * (z - H_ * statePred.x);
	stateOut.P = statePred.P - K * H_ * statePred.P;
	return stateOut;
}

