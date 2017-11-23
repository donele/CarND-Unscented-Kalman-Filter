#ifndef UKFLaser_H_
#define UKFLaser_H_
#include "Eigen/Dense"
#include "tools.h"
#include "UKF.h"

class UKFLaser: public UKF {
public:
  /**
  * Constructor
  */
  UKFLaser(float std_a, float std_yawdd);

  /**
  * Destructor
  */
  virtual ~UKFLaser();

private:
  int n_z_;
  float std_laspx_;
  float std_laspy_;
  Tools tools;
  Eigen::MatrixXd Zsig_;
  Eigen::MatrixXd Zpred_;
  Eigen::MatrixXd S_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Tc_;
  Eigen::MatrixXd Diff_z_;
  Eigen::MatrixXd K_;

  /**
   * Updates the state by using Kalman Filter equations
   * @param state state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
   */
  virtual void Update(StateCTRV& state, float dt, const Eigen::VectorXd& z);
};

#endif
