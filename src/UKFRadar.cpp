#include "UKFRadar.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKFRadar::UKFRadar(float std_a, float std_yawdd)
  :UKF(std_a, std_yawdd),
  n_z_(3),
  std_radr_(.3),
  std_radphi_(.03),
  std_radrd_(.3)
{
  Zsig_ = MatrixXd(n_z_, 2 * n_aug + 1);
  Zpred_ = VectorXd(n_z_);
  S_ = MatrixXd(n_z_, n_z_);
  Tc_ = MatrixXd(n_x, n_z_);

  // measurement noise
  R_ = MatrixXd(n_z_, n_z_);
  R_.setZero();
  R_.diagonal() << std_radr_*std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_;
}

UKFRadar::~UKFRadar() {}

void UKFRadar::Update(StateCTRV& state, float dt, const VectorXd &z) {
  // Transform sigma points into measurement space
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    float px = Xsig_pred_.col(i)[0];
    float py = Xsig_pred_.col(i)[1];
    float v = Xsig_pred_.col(i)[2];
    float psi = Xsig_pred_.col(i)[3];

    float rho = sqrt(px*px + py*py);
    float phi = tools.NormalizePi(atan2(py, px));
    float rho_dot = (px * cos(psi) * v + py * sin(psi) * v) / rho;
    Zsig_.col(i) << rho, phi, rho_dot;
  }

  // Calculate mean predicted measurement
  Zpred_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Zpred_ += weights_[i] * Zsig_.col(i);
  }
  Zpred_(1) = tools.NormalizePi(Zpred_(1));

  // Calculate measurement covariance matrix S
  S_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_z_ = Zsig_.col(i) - Zpred_;
    Diff_z_(1) = tools.NormalizePi(Diff_z_(1));
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
    Diff_z_(1) = tools.NormalizePi(Diff_z_(1));

    Tc_ += weights_[i] * Diff_ * Diff_z_.transpose();
  }

  // Calculate Kalman gain K
  K_ = Tc_ * S_.inverse();

  // Update state mean and covariance matrix
  Diff_z_ = z - Zpred_;
  Diff_z_(1) = tools.NormalizePi(Diff_z_(1));
  state.x += K_ * Diff_z_;
  state.x(3) = tools.NormalizePi(state.x(3));

  state.P -= K_ * S_ * K_.transpose();

  cout << "Radar update";
  printf(" %.4f %.4f %.4f\n", z(0), z(1), z(2));
  cout << state.x << endl;
  cout << state.P << endl;

  return;
}
