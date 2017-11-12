#ifndef KFLaser_H_
#define KFLaser_H_
#include "Eigen/Dense"
#include "KF.h"

class KFLaser: public KF {
public:

  // measurement matrix
  Eigen::MatrixXd H_;
  Eigen::MatrixXd H_trans_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
  * Constructor
  */
  KFLaser();

  /**
  * Destructor
  */
  virtual ~KFLaser();

  /**
   * Updates the state by using Kalman Filter equations
   * @param state state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
   */
  virtual void Update(KFState& state, float dt, const Eigen::VectorXd& z);
};

#endif
