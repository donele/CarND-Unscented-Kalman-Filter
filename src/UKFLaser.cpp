#include "KFLaser.h"

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
        0,      0.0225;
}

KFLaser::~KFLaser() {}

void KFLaser::Update(KFState& state, float dt, const VectorXd &z) {
  // Calculate Kalman gain
  MatrixXd K = state.P * H_trans_ * (H_ * state.P * H_trans_ + R_).inverse();

  // Update the state from the measurement.
  state.x = state.x + K * (z - H_ * state.x);
  state.P = state.P - K * H_ * state.P;
  return;
}

