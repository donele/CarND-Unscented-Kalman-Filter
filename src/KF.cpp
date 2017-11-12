#include "KF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;

KF::KF()
:noise_a_(9.),
noise_y_(9.) {
  F_ = MatrixXd::Zero(4, 4);
  Q_ = MatrixXd::Zero(4, 4);
}

KF::~KF() {}

void KF::Predict(KFState& state, float dt) {
  // Do no predict if dt is very small.
  if(dt < 1e-6)
    return;

  // Prediction calculation is shared by all subclasses.
  set_F(dt);
  set_Q(dt);
  state.x = F_ * state.x;
  state.P = F_ * state.P * F_.transpose() + Q_;

  return;
}

void KF::set_F(float dt) {
  // Calculate state transition matrix F according to the elapsed time.
  F_ = MatrixXd::Identity(4, 4);
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KF::set_Q(float dt) {
  // Calculate process covariance according to the elapsed time.
  float noise_x = 9;
  float noise_y = 9;
  float dt2 = dt * dt;
  float dt3 = dt * dt2;
  float dt4 = dt2 * dt2;
  float Q13 = noise_x*dt3/2;
  float Q24 = noise_y*dt3/2;
  Q_ << noise_x*dt4/4, 0,             Q13,         0,
        0,             noise_y*dt4/4, 0,           Q24,
        Q13,           0,             noise_x*dt2, 0,
        0,             Q24,           0,           noise_y*dt2;
}
