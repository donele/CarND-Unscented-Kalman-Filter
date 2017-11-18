#ifndef UKFLaser_H_
#define UKFLaser_H_
#include "Eigen/Dense"
#include "UKF.h"

class UKFLaser: public UKF {
public:

  // measurement matrix
  Eigen::MatrixXd H_;
  Eigen::MatrixXd H_trans_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
  * Constructor
  */
  UKFLaser();

  /**
  * Destructor
  */
  virtual ~UKFLaser();

private:
  /**
   * Updates the state by using Kalman Filter equations
   * @param state state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
   */
  virtual void Update(StateCTRV& state, float dt, const Eigen::VectorXd& z);
};

#endif
