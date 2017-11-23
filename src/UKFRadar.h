#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"
#include "UKF.h"
#include "StateCTRV.h"

class UKFRadar: public UKF {
public:
  /**
  * Constructor
  */
  UKFRadar(float std_a, float std_yawdd);

  /**
  * Destructor
  */
  virtual ~UKFRadar();

private:
  int n_z_;
  float std_radr_;
  float std_radphi_;
  float std_radrd_;
  Tools tools;
  Eigen::MatrixXd Zsig_;
  Eigen::VectorXd Zpred_;
  Eigen::MatrixXd S_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Tc_;
  Eigen::MatrixXd Diff_z_;
  Eigen::MatrixXd K_;

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param state state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
   */
  void Update(StateCTRV& state, float dt, const Eigen::VectorXd &z);
};

#endif
