#include "UKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF(float std_a, float std_yawdd)
  :n_x(5),
  n_aug(7),
  std_a_(std_a),
  std_yawdd_(std_yawdd)
{
    lambda = 3 - n_aug;
    Xaug_ = VectorXd(n_aug);
    Paug_ = MatrixXd(n_aug, n_aug);
    Xsig_aug_ = MatrixXd(n_aug, 2 * n_aug + 1);
    Xsig_motion_ = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_noise_ = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred_ = MatrixXd(n_x, 2 * n_aug + 1);
    weights_ = VectorXd(2 * n_aug + 1);
    weights_(0) = lambda / (lambda + n_aug);
    weights_.tail(2 * n_aug) = VectorXd::Zero(2 * n_aug).array() + .5 / (lambda + n_aug);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(StateCTRV& state, float dt, const Eigen::VectorXd& z) {
  Predict(state, dt);
  Update(state, dt, z);
}

void UKF::Predict(StateCTRV& state, float dt) {
  // Calculate Xaug
  Xaug_.head(n_x) = state.x;
  Xaug_.tail(n_aug - n_x) = VectorXd::Zero(n_aug - n_x);

  // Calculate Paug
  Paug_.setZero();
  Paug_.topLeftCorner(n_x, n_x) = state.P;
  Paug_.bottomRightCorner(n_aug - n_x, n_aug - n_x).diagonal() << std_a_*std_a_, std_yawdd_*std_yawdd_;

  // Calculate sigma points
  SqrtP_ = Paug_.llt().matrixL();
  Xsig_aug_.setZero();
  Xsig_aug_.col(0) = Xaug_;
  for(int i = 0; i < n_aug; ++i) {
    Xsig_aug_.col(i + 1) = Xaug_ + sqrt(lambda + n_aug) * SqrtP_.col(i);
    Xsig_aug_.col(n_aug + i + 1) = Xaug_ - sqrt(lambda + n_aug) * SqrtP_.col(i);
  }

  // Calculate sigma points predictions
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    float v = Xsig_aug_.col(i)[2];
    float psi = Xsig_aug_.col(i)[3];
    float psi_dot = Xsig_aug_.col(i)[4];
    float nu_a = Xsig_aug_.col(i)[5];
    float nu_psi = Xsig_aug_.col(i)[6];
    if(psi_dot < 1e-2 && psi_dot > -1e-2) {
      Xsig_motion_.col(i) << v * cos(psi) * dt,
                             v * sin(psi) * dt,
                             0.,
                             psi_dot * dt,
                             0.;
      Xsig_noise_.col(i) << .5 * nu_a * dt * dt * cos(psi),
                            .5 * nu_a * dt * dt * sin(psi),
                            dt * nu_a,
                            .5 * dt * dt * nu_psi,
                            dt * nu_psi;
    }
    else {
      Xsig_motion_.col(i) << v / psi_dot * (sin(psi + psi_dot * dt) - sin(psi)),
                             v / psi_dot * (-cos(psi + psi_dot * dt) + cos(psi)),
                             0.,
                             psi_dot * dt,
                             0.;
      Xsig_noise_.col(i) << .5 * nu_a * dt * dt * cos(psi),
                            .5 * nu_a * dt * dt * sin(psi),
                            dt * nu_a,
                            .5 * dt * dt * nu_psi,
                            dt * nu_psi;
    }
  }

  Xsig_pred_ = Xsig_aug_.topRows(n_x) + Xsig_motion_ + Xsig_noise_;

  // Calculate predicted state
  state.x.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    state.x += weights_(i) * Xsig_pred_.col(i);
  }

  state.P.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_ = Xsig_pred_.col(i) - state.x;
    state.P += weights_(i) * Diff_ * Diff_.transpose();
  }
  cout << "predict" << endl;
  cout << state.x << endl;
  cout << state.P << endl;
}
