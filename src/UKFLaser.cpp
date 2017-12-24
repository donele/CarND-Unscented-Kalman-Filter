#include "UKFLaser.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKFLaser::UKFLaser(float std_a, float std_yawdd)
  :UKF(std_a, std_yawdd),
  n_z_(2),
  std_laspx_(0.15),
  std_laspy_(0.15)
{
  Zsig_ = MatrixXd(n_z_, 2 * n_aug + 1);
  Zpred_ = VectorXd(n_z_);
  S_ = MatrixXd(n_z_, n_z_);
  Tc_ = MatrixXd(n_x, n_z_);

  // measurement noise
  R_ = MatrixXd(n_z_, n_z_);
  R_.setZero();
  R_.diagonal() << std_laspx_*std_laspx_, std_laspy_*std_laspy_;
}

UKFLaser::~UKFLaser() {}

void UKFLaser::Update(StateCTRV& state, float dt, const VectorXd &z) {
  // Transform sigma points into measurement space
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    float px = Xsig_pred_.col(i)[0];
    float py = Xsig_pred_.col(i)[1];

    Zsig_.col(i) << px, py;
  }

  // Calculate mean predicted measurement
  Zpred_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Zpred_ += weights_[i] * Zsig_.col(i);
  }

  // Calculate measurement covariance matrix S
  S_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_z_ = Zsig_.col(i) - Zpred_;
    S_ += weights_[i] * Diff_z_ * Diff_z_.transpose();
  }
  S_ += R_;

  // Calculate NIS
  NIS = (z - Zpred_).transpose() * S_.inverse() * (z - Zpred_);

  // Calculate cross correlation matrix
  Tc_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_ = Xsig_pred_.col(i).head(n_x) - state.x;
    Diff_(3) = tools.NormalizePi(Diff_(3));
    Diff_z_ = Zsig_.col(i) - Zpred_;
    Tc_ += weights_[i] * Diff_ * Diff_z_.transpose();
  }

  // Calculate Kalman gain K
  K_ = Tc_ * S_.inverse();

  // Update state mean and covariance matrix
  Diff_z_ = z - Zpred_;
  //Diff_z_(1) = tools.NormalizePi(Diff_z_(1));
  state.x += K_ * Diff_z_;
  state.x(3) = tools.NormalizePi(state.x(3));

  state.P -= K_ * S_ * K_.transpose();

  cout << "Laser update";
  printf(" %.4f %.4f\n", z(0), z(1));
  cout << state.x << endl;
  cout << state.P << endl;

  return;
}

