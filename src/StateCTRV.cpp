#include "StateCTRV.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

StateCTRV::StateCTRV() {
  x = VectorXd::Zero(5);
  P = MatrixXd(5, 5);
}

StateCTRV::~StateCTRV() {}
