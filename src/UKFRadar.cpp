#include "UKFRadar.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UKFRadar::UKFRadar()
  :n_z_(3),
  std_radr_(.3),
  std_radphi_(.03),
  std_radrd_(.3)
{
  Zsig_ = MatrixXd(n_z_, 2 * n_aug + 1);
  Zpred_ = MatrixXd(n_z_, 2 * n_aug + 1);
  S_ = MatrixXd(n_z_, n_z_);
  Tc_ = MatrixXd(n_z_, n_z_);

  // measurement noise
  R_ = MatrixXd(n_z_, n_z_);
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

  // Calculate measurement covariance matrix S
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_z_ = Zsig_.col(i) - Zpred_;
    Diff_z_(1) = tools.NormalizePi(Diff_z_(1));
    S_ += weights_[i] * Diff_z_ * Diff_z_.transpose();
  }
  S_ += R_;

  // Calculate cross correlation matrix
  Tc_.setZero();
  for(int i = 0; i < 2 * n_aug + 1; ++i) {
    Diff_ = Xsig_pred_.col(i).head(n_x) - state.x;
    Diff_(1) = tools.NormalizePi(Diff_(1));

    Diff_z_ = Zsig_.col(i) - Zpred_;
    Diff_z_(1) = tools.NormalizePi(Diff_z_(1));

    Tc_ += weights_[i] * Diff_ * Diff_z_.transpose();
  }

  // Calculate Kalman gain K
  K_ = Tc_ * S_.inverse();

  // Update state mean and covariance matrix
  state.x += K_ * (z - Zpred_);
  state.P -= K_ * S_ * K_.transpose();

  return;
}
