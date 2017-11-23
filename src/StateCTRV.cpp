#include "StateCTRV.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

StateCTRV::StateCTRV() {
  // px, py, v, psi, psi_dot
  x = VectorXd::Zero(5);

  P = MatrixXd(5, 5);
  P << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, 0, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;
}

StateCTRV::~StateCTRV() {}

VectorXd StateCTRV::GetStateCV() {
  VectorXd x_cv(4);
  x_cv << x(0), x(1), x(2) * cos(x(3)), x(2) * sin(x(3));
  return x_cv;
}
