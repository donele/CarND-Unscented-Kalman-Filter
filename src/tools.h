#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
const double PI = 3.14159265;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to convert Cartesian coordinates to polar.
  */
  VectorXd Cartesian2Polar(const VectorXd& x_state);

  /**
  * A helper method to convert Polar coordinates to Cartesian.
  */
  VectorXd Polar2Cartesian(const VectorXd& x_state);

  float NormalizePi(float angle);
};

#endif /* TOOLS_H_ */
