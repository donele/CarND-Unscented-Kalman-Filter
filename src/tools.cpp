#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  int ne = estimations.size();
  int nt = ground_truth.size();
  if(ne == 0 || nt == 0 || nt != nt)
    return rmse;

  for(int i = 0; i < nt; ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    rmse = rmse.array() + diff.array() * diff.array();
  }
  rmse /= ne;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float c = sqrt(px*px + py*py);
  float c2 = c*c;
  float c3 = c*c2;

  MatrixXd jacobian(3, 4);
  jacobian << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
  if(c > 1e-6) {
    jacobian << px/c,                  py/c,                  0,    0,
                -py/c2,                px/c2,                 0,    0,
                py*(vx*py - px*vy)/c3, px*(vy*px - py*vx)/c3, px/c, py/c;
  }
  return jacobian;
}

VectorXd Tools::Cartesian2Polar(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float c = sqrt(px*px + py*py);

  VectorXd polar_state(3);
  polar_state(0) = c;
  polar_state(1) = atan2(py, px);
  polar_state(2) = (c > 1e-6) ? (px*vx + py*vy) / c : 0;
  return polar_state;
}

VectorXd Tools::Polar2Cartesian(const VectorXd& x_state) {
  float ro = x_state(0);
  float theta = x_state(1);
  float ro_dot = x_state(2);

  VectorXd cart_state(4);
  cart_state(0) = ro * cos(theta);
  cart_state(1) = ro * sin(theta);
  cart_state(2) = ro_dot * cos(theta);
  cart_state(3) = ro_dot * sin(theta);
  return cart_state;
}
