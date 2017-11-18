#ifndef UKF_H_
#define UKF_H_
#include "Eigen/Dense"
#include "StateCTRV.h"

class UKF {
public:

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
  * Constructor
  */
  UKF();

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
  float noise_a_;
  float noise_y_;

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

  /**
  * Calculate state transition matrix F
  * @param dt elapsed time from k to k+1, in seconds
  */
  void set_F(float dt);

  /**
  * Calculate process covariance
  * @param dt elapsed time from k to k+1, in seconds
  */
  void set_Q(float dt);
};

#endif
