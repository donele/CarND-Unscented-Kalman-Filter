#ifndef KF_H_
#define KF_H_
#include "Eigen/Dense"
#include "KFState.h"

class KF {
public:

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

	/**
	 * Constructor
	 */
  KF();

	/**
	 * Destructor
	 */
  virtual ~KF();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
	 * @param stateIn state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   */
  KFState Predict(const KFState& stateIn, float dt);

	/**
	 * Subclasses must define how to update the states
	 * @param stateIn state vector and covariance prior to update
   * @param dt elapsed time from k to k+1, in seconds
   * @param z The measurement at k+1
	 */
	virtual KFState Update(const KFState& stateIn, float dt, const Eigen::VectorXd& z) = 0;

private:
	float noise_a_;
	float noise_y_;

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
