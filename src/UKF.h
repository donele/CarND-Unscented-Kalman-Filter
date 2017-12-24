#ifndef UKF_H_
#define UKF_H_
#include "Eigen/Dense"
#include "tools.h"
#include "StateCTRV.h"

class UKF {
public:
  int n_x;
  int n_aug;
  double lambda;
  float NIS;

  /**
  * Constructor
  */
  UKF(float std_a, float std_yawdd);

  /**
  * Destructor
  */
  virtual ~UKF();

  /**
  * Process measurement by calling Predict() and Update()
  * @param state state vector and covariance prior to update
  * @param dt elapsed time from k to k+1, in seconds
  * @param z The measurement at k+1
  */
  void ProcessMeasurement(StateCTRV& state, float dt, const Eigen::VectorXd& z);

protected:
  Tools tools;
  float std_a_;
  float std_yawdd_;

  // sigma point
  Eigen::VectorXd Xaug_;
  Eigen::MatrixXd Paug_;
  Eigen::MatrixXd SqrtP_;
  Eigen::MatrixXd Xsig_aug_;
  Eigen::MatrixXd Xsig_motion_;
  Eigen::MatrixXd Xsig_noise_;
  Eigen::MatrixXd Xsig_pred_;
  Eigen::VectorXd weights_;
  Eigen::VectorXd Diff_;

  /**
  * Prediction Predicts the state and the state covariance
  * using the process model
  * @param state state vector and covariance prior to update
  * @param dt elapsed time from k to k+1, in seconds
  */
  void Predict(StateCTRV& state, float dt);

  /**
  * Subclasses must define how to update the states
  * @param state state vector and covariance prior to update
  * @param dt elapsed time from k to k+1, in seconds
  * @param z The measurement at k+1
  */
  virtual void Update(StateCTRV& state, float dt, const Eigen::VectorXd& z) = 0;
};

#endif
