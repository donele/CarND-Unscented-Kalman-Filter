#include "UKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF()
  :n_x(5),
  n_aug(7),
  std_a_(30),
  std_yawdd_(30) {
    lambda = 3 - n_aug;
    Xsig_aug_ = MatrixXd(n_aug, 2 * n_aug + 1);
    Xsig_pred_ = MatrixXd(n_aug, 2 * n_aug + 1);
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
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Xsig_aug_.col(i + 1) = Xaug_ + sqrt(lambda + n_x) * SqrtP_.col(i);
    Xsig_aug_.col(n_x + i + 1) = Xaug_ - sqrt(lambda + n_x) * SqrtP_.col(i);
  }

  // Calculate sigma points predictions
  Xsig_pred_.setZero();

  float px = state.x[0];
  float py = state.x[1];
  float v = state.x[2];
  float psi = state.x[3];
  float psi_dot = state.x[4];
  if(psi_dot < 1e-6 && psi_dot > -1e-6) {
    for(int i = 0; i < 2 * n_aug + 1; ++i) {
      Xsig_process_time_.col(i) << v * cos(psi) * dt,
                                   v * sin(psi) * dt,
                                   0.,
                                   psi_dot * dt,
                                   0.;
      Xsig_process_noise_.col(i) << .5 * dt * dt * cos(psi) + std_a_,
                                    .5 * dt * dt * sin(psi) + std_a_,
                                    dt * std_a_,
                                    .5 * dt * dt * std_yawdd_,
                                    dt * std_yawdd_;
    }
  }
  else {
    for(int i = 0; i < 2 * n_aug + 1; ++i) {
      Xsig_process_time_.col(i) << v / psi_dot * (sin(psi + psi_dot *dt) - sin(psi)),
                                   v / psi_dot * (-cos(psi + psi_dot * dt) + cos(psi)),
                                   0.,
                                   psi_dot * dt,
                                   0.;
      Xsig_process_noise_.col(i) << .5 * dt * dt * cos(psi) + std_a_,
                                    .5 * dt * dt * sin(psi) + std_a_,
                                    dt * std_a_,
                                    .5 * dt * dt * std_yawdd_,
                                    dt * std_yawdd_;
    }
  }

  Xsig_pred_ = Xsig_aug_ + Xsig_process_time_ + Xsig_process_noise_;

  // Calculate predicted state
  state.x.setZero();
  state.P.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    state.x += weights_(i) * Xsig_pred_.col(i);
    Diff_ = Xsig_pred_.col(i) - state.x;
    state.P += weights_(i) * Diff_ * Diff_.transpose();
  }
}
