#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"
#include "KF.h"
#include "KFState.h"

class KFRadar: public KF {
public:

  // measurement matrix
  Eigen::MatrixXd H_;
  Eigen::MatrixXd H_trans_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
  * Constructor
  */
  KFRadar();

  /**
  * Destructor
  */
  virtual ~KFRadar();

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param state state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
   */
  void Update(KFState& state, float dt, const Eigen::VectorXd &z);

private:
  Tools tools;
};

#endif
