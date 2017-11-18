#include "UKFRadar.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
const double PI = 3.14159265;

UKFRadar::UKFRadar() {
  // measurement matrix
  H_ = MatrixXd::Zero(3, 4);
  H_trans_ = MatrixXd::Zero(4, 3);

  // measurement noise
  R_ = MatrixXd(3, 3);
  R_ << 0.09, 0,      0,
        0,    0.0009, 0,
        0,    0,      0.09;
}

UKFRadar::~UKFRadar() {}

void UKFRadar::Update(StateCTRV& state, float dt, const VectorXd &z) {
  // Keep the value of theta in [-pi, pi]
  VectorXd y(3);
  y = z - tools.Cartesian2Polar(state.x);
  float theta = y[1];
  while(theta > PI)
    theta -= 2*PI;
  while(theta < -PI)
    theta += 2*PI;
  y[1] = theta;

  // Calculate jacobian
  H_ = tools.CalculateJacobian(state.x);
  H_trans_ = H_.transpose();

  // Calculate Kalman gain
  MatrixXd K = state.P * H_trans_ * (H_ * state.P * H_trans_ + R_).inverse();

  // Update state and covariance matrices
  state.x = state.x + K * y;
  state.P = state.P - K * H_ * state.P;
  return;
}
