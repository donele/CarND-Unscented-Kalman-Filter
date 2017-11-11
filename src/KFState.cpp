#include "KFState.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KFState::KFState() {
	x = VectorXd::Zero(4);
	P = MatrixXd(4, 4);
	P << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 10, 0,
				0, 0, 0, 10;
}

KFState::~KFState() {}
